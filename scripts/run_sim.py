"""启动右臂操作水龙头。先创建 SimulationApp，再导入任何仿真 API。"""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback
from datetime import datetime


def main():
    """解析启动参数；失败返回非零退出码，窗口默认在任务结束后保留。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='无窗口运行物理验收')
    parser.add_argument('--quit-on-complete', action='store_true', help='流程结束后退出')
    parser.add_argument('--realtime', action='store_true', help='按仿真时间限速，方便观察')
    parser.add_argument('--config', help='自定义 JSON 配置文件')
    parser.add_argument('--output', help='输出目录，默认使用时间戳')
    parser.add_argument('--capture-frames', action='store_true', help='保存各阶段的实际渲染截图')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    output = Path(args.output or root / 'outputs' / datetime.now().strftime('%Y%m%d_%H%M%S')).resolve()
    output.mkdir(parents=True, exist_ok=True)
    # 覆盖指定目录重跑时先使旧成功标记失效，避免初始化失败仍留下旧 passed=true。
    result_path = output / 'result.json'
    result_path.write_text(json.dumps({'passed': False, 'status': 'starting'}), encoding='utf-8')
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 900,
                         'anti_aliasing': 0, 'renderer': 'RaytracedLighting'})
    simulation, code = None, 0
    try:
        from faucet_sim.config import load_config
        from faucet_sim.runtime import FaucetSimulation
        config = load_config(args.config)
        (output / 'effective_config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
        simulation = FaucetSimulation(app, config, output, args.headless, args.realtime, args.capture_frames)
        result = simulation.execute()
        simulation.save(result)
        print(f'[完成] 验收通过，结果目录：{simulation.output}', flush=True)
        if not args.headless and not args.quit_on_complete:
            while app.is_running():
                simulation.step()
    except Exception as error:
        code = 1
        traceback.print_exc()
        if simulation is not None:
            simulation.save({'passed': False, 'error': str(error)})
            if args.capture_frames:
                simulation.capture_frame('failure.png')
                simulation.capture_grasp_detail('failure_detail.png')
        else:
            result_path.write_text(json.dumps({'passed': False, 'error': str(error)}, ensure_ascii=False, indent=2),
                                   encoding='utf-8')
    finally:
        if simulation is not None:
            simulation.world.stop()
        app.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
