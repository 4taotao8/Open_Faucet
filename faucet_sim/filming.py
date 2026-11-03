"""仿真影片：按任务阶段调度摄影机，读取真实 RGB 帧并编码为 MP4。

仅控制相机、灯光和画面标注，不修改机器人命令、碰撞或水龙头状态。
使用环境已有的 Pillow、Isaac 相机和系统 FFmpeg；不会下载编码器。
"""
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, UsdGeom, UsdLux, UsdShade, Sdf
from scipy.spatial.transform import Rotation
from isaacsim.sensors.camera import Camera
from .geometry import smoothstep
from .visuals import water_state


class DemoFilm:
    """一个真实物理回合的分镜导演、中文字幕层和无损帧管道。"""

    def __init__(self, simulation, destination, width=1920, height=1080, fps=30):
        """输入运行中的仿真、输出目录和画质；生成 H.264 MP4 与分镜记录。"""
        self.simulation, self.width, self.height, self.fps = simulation, width, height, fps
        self.folder = Path(destination)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / 'faucet_multiview.mp4'
        self.frames, self.shots = 0, []
        self.phase, self.kind, self.index = '机械臂开关水龙头', 'intro', -1
        self.phase_time, self.phase_duration = simulation.sim_time, 3.
        self.next_frame = simulation.sim_time
        self.previous_rgb = None
        self.cut_frame = 0
        self._presentation()
        self.camera = Camera('/World/FilmCamera', resolution=(width, height), annotator_device='cpu')
        self.camera.initialize()
        self.camera.prim.GetAttribute('horizontalAperture').Set(36.)
        self.camera.prim.GetAttribute('verticalAperture').Set(36.*height/width)
        self.camera.set_clipping_range(.01, 100.)
        self.camera.set_lens_aperture(0.)
        font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
        if not Path(font_path).exists():
            raise RuntimeError('缺少中文字体：请指定已有的 NotoSansCJK 字体，不会自动安装')
        scale = width/1920
        self.fonts = {name: ImageFont.truetype(font_path, int(size*scale)) for name, size in
                      [('title', 64), ('phase', 40), ('body', 27), ('small', 21), ('number', 30)]}
        ffmpeg = shutil.which('ffmpeg')
        if not ffmpeg:
            raise RuntimeError('系统没有 FFmpeg，录制前需先取得安装许可')
        self.encoder_log = (self.folder / 'encoding.log').open('w', encoding='utf-8')
        self.encoder = subprocess.Popen([
            ffmpeg, '-hide_banner', '-loglevel', 'warning', '-y', '-f', 'rawvideo',
            '-pixel_format', 'rgb24', '-video_size', f'{width}x{height}', '-framerate', str(fps),
            '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-threads', '4', '-preset', 'fast', '-crf', '18',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(self.path)
        ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.encoder_log)
        self._set_view(0.)
        for _ in range(30):
            simulation.world.render()
        rgb = self.camera.get_rgb()
        if rgb is None or rgb.shape[:2] != (height, width):
            raise RuntimeError('相机没有返回有效画面')
        self.intro()

    def _presentation(self):
        """设置柔和灯光与哑光背景；不修改物理材质或碰撞配置。"""
        stage = self.simulation.stage
        for name, color in [('Floor', (.055, .08, .12)), ('Counter', (.25, .31, .36)),
                            ('RobotStand', (.10, .14, .19))]:
            material = UsdShade.Material.Define(stage, '/World/FilmMaterials/' + name)
            shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + '/Surface')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.7)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/' + name)).Bind(material)
        light = UsdLux.DistantLight(stage.GetPrimAtPath('/World/KeyLight'))
        light.CreateAngleAttr(7.)
        light.CreateIntensityAttr(1800.)
        UsdLux.DomeLight(stage.GetPrimAtPath('/World/Sky')).CreateIntensityAttr(500.)

    def phase_start(self, simulation, segment, index):
        """记录阶段与分镜边界，切镜不改变物理回合。"""
        self.phase, self.kind, self.index = segment.name, segment.kind, index
        self.phase_time = simulation.sim_time
        self.phase_duration = float(segment.time[-1]) + .5
        self.cut_frame = self.frames
        self.shots.append({'phase': self.phase, 'kind': self.kind, 'start_video_s': self.frames/self.fps})
        print(f'[录制] {index+1:02d} {segment.name}', flush=True)

    def _set_view(self, progress):
        """依据阶段设置全景、侧面、俯视或夹持特写，并作缓慢移动。"""
        p = smoothstep(float(np.clip(progress, 0, 1)))
        right, valve = self.simulation.measured()
        handle = self.simulation.faucet_model.target(*valve)[:3, 3]
        if self.kind in ['intro', 'transit', 'home', 'outro']:
            target = np.array([.12, .22, .64])
            eye = np.array([-1.00 + .16*p, -.95 + .12*p, 1.04 + .05*p])
            focal = 35.
        elif self.kind in ['approach', 'retreat']:
            target = np.array([0., .14, .72])
            eye = np.array([-.58 + .08*p, -.05 + .05*p, .94])
            focal = 35.
        elif self.kind in ['grasp', 'release']:
            target = handle + np.array([0., .045, -.012])
            eye = target + np.array([-.29 + .025*p, .01, .085])
            focal = 31.
        elif self.index in [4, 5]:
            # 俯视观察 Z 轴调温；相机从右臂外侧拍摄，避开左臂遮挡。
            target = handle + np.array([0., .035, -.012])
            eye = target + np.array([-.28 + .04*p, -.16 + .04*p, .36])
            focal = 35.
        elif self.index in [6, 7]:
            # 侧面近景可以直接看到把手抬升与夹爪同步转动。
            target = handle + np.array([0., .04, -.025])
            eye = target + np.array([-.39 + .03*p, .025, .10])
            focal = 32.
        else:
            target = np.array([0., .115, .66])
            eye = target + np.array([-.52 + .04*p, -.26, .28])
            focal = 34.
        matrix = np.array(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target),
                                               Gf.Vec3d(0, 0, 1)).GetInverse()).T
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        self.camera.set_world_pose(eye, quaternion[[3, 0, 1, 2]], camera_axes='usd')
        self.camera.prim.GetAttribute('focalLength').Set(focal)

    def after_step(self, simulation):
        """按仿真时间采样视频帧，墙钟运行变慢不会改变影片帧率。"""
        if simulation.sim_time + 1e-8 < self.next_frame:
            return
        progress = (simulation.sim_time-self.phase_time) / self.phase_duration
        self._set_view(progress)
        self._capture()
        self.next_frame += 1/self.fps

    def _capture(self):
        """刷新摄影机并直接将 RGB 帧送入 FFmpeg，避免存储海量临时 PNG。"""
        for _ in range(2):
            self.simulation.world.render()
        rgb = self.camera.get_rgb()
        if rgb is None or rgb.shape != (self.height, self.width, 3):
            raise RuntimeError('录制中丢失相机 RGB 数据')
        rgb = np.asarray(rgb, dtype=np.uint8).copy()
        # 切镜后 6 帧轻微交叠，避免特写与全景之间的生硬闪切。
        age = self.frames-self.cut_frame
        if 0 <= age < 6 and self.previous_rgb is not None:
            fraction = (age+1)/6
            displayed = (self.previous_rgb*(1-fraction) + rgb*fraction).astype(np.uint8)
        else:
            displayed = rgb
        if age >= 6 or self.previous_rgb is None:
            self.previous_rgb = rgb
        frame = self._overlay(Image.fromarray(displayed).convert('RGBA')).convert('RGB')
        self.encoder.stdin.write(frame.tobytes())
        if self.frames == 15 or (self.index >= 0 and age == 12):
            frame.save(self.folder / f'preview_{self.index+1:02d}.jpg', quality=94)
        self.frames += 1

    def _overlay(self, image):
        """绘制中文阶段、实际阀角和清晰的播放进度；无应用界面遮挡。"""
        scale = self.width/1920
        layer = Image.new('RGBA', image.size)
        draw = ImageDraw.Draw(layer)
        def box(bounds, color, radius=16):
            draw.rounded_rectangle(tuple(int(v*scale) for v in bounds), radius=int(radius*scale), fill=color)
        def text(x, y, value, font='body', color=(240, 246, 255, 255)):
            draw.text((int(x*scale), int(y*scale)), value, font=self.fonts[font], fill=color)
        if self.kind == 'intro':
            box((1000, 55, 1865, 260), (10, 22, 36, 226), 22)
            text(1030, 78, '机械臂开关水龙头', 'title')
            text(1034, 176, '正面接近 / 上下夹持 / 双自由度操作', color=(132, 218, 238, 255))
        elif self.kind == 'outro':
            box((55, 55, 850, 225), (10, 22, 36, 226), 22)
            text(88, 78, '操作完成 · 返回初始位置', 'phase')
            text(90, 148, '完整流程验收通过', color=(132, 232, 192, 255))
        else:
            box((55, 55, 850, 180), (10, 22, 36, 222))
            text(82, 74, f'{self.index+1:02d} / 12', 'small', (126, 215, 235, 255))
            text(82, 108, self.phase, 'phase')
        _, valve = self.simulation.measured()
        state = water_state(*valve)
        box((55, 943, 1865, 1045), (9, 21, 35, 229))
        text(86, 964, f'左右角  {np.rad2deg(valve[0]):+.1f}°')
        text(460, 964, f'抬升角  {np.rad2deg(valve[1]):.1f}°')
        text(810, 964, f"流量  {state['flow_lpm']:.2f} L/min")
        bilateral = bool(self.simulation.contacts.last.get('fixed_contact_points') and
                         self.simulation.contacts.last.get('moving_contact_points'))
        holding = self.simulation.grasp.active and bilateral
        label = '接触夹持' if holding else '接触丢失' if self.simulation.grasp.active else '夹爪释放'
        text(1210, 964, label, color=(132, 232, 192, 255) if holding else (230, 200, 170, 255))
        text(1510, 967, '真实物理回合', 'small', (169, 192, 214, 255))
        text(65, 905, '纯接触摩擦交互 · 无固定抓取约束 · 水流为示意', 'small', (220, 232, 242, 255))
        if self.index >= 0:
            fraction = (self.index + np.clip((self.simulation.sim_time-self.phase_time)/self.phase_duration, 0, 1))/12
            draw.rectangle((int(55*scale), int(1038*scale), int((55+1810*fraction)*scale),
                            int(1045*scale)), fill=(90, 213, 226, 255))
        return Image.alpha_composite(image, layer)

    def intro(self):
        """录制三秒片头；仅移动摄影机，物理状态保持不变。"""
        for index in range(self.fps*3):
            self._set_view(index/(self.fps*3))
            self._capture()

    def finish(self):
        """录制三秒片尾、完成容器封装并保存分镜元数据。"""
        self.kind, self.phase = 'outro', '操作完成'
        self.cut_frame = self.frames
        for index in range(self.fps*3):
            self._set_view(index/(self.fps*3))
            self._capture()
        self.close()
        metadata = {'video': self.path.name, 'resolution': [self.width, self.height], 'fps': self.fps,
                    'frames': self.frames, 'duration_s': self.frames/self.fps, 'shots': self.shots,
                    'source': 'single measured PhysX episode; rendered camera frames', 'audio': False}
        (self.folder / 'video_info.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
        return metadata

    def close(self):
        """关闭编码管道；失败时也保留已录制片段供排查。"""
        if self.encoder.poll() is None:
            self.encoder.stdin.close()
            code = self.encoder.wait(timeout=60)
            if code:
                raise RuntimeError(f'FFmpeg 编码失败，退出码 {code}')
        self.encoder_log.close()
