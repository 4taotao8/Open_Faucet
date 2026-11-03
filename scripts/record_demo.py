"""录制多视角 1080p 水龙头操作影片；直接录制真实物理回合。"""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback


def main():
    """启动应用、执行完整回合并保存影片和物理验收结果。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='outputs/videos/faucet_multiview')
    parser.add_argument('--config')
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--preview-only', action='store_true', help='只输出各阶段机位预览，不执行动作')
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 960, 'height': 540, 'anti_aliasing': 3,
                         'renderer': 'RaytracedLighting'})
    film, simulation, code = None, None, 0
    try:
        from faucet_sim.config import load_config
        from faucet_sim.runtime import FaucetSimulation
        from faucet_sim.filming import DemoFilm
        output = Path(args.output).resolve()
        simulation = FaucetSimulation(app, load_config(args.config), output / 'simulation', headless=True)
        film = DemoFilm(simulation, output, args.width, args.height, args.fps)
        if args.preview_only:
            for index, segment in enumerate(simulation.segments):
                film.phase_start(simulation, segment, index)
                for frame in range(14):
                    film._set_view(frame/14)
                    film._capture()
            film.close()
            print(f'[预览] {output}', flush=True)
        else:
            simulation.observer = film
            result = simulation.execute()
            simulation.observer = None
            simulation.save(result)
            metadata = film.finish()
            print('[完成] ' + json.dumps(metadata, ensure_ascii=False), flush=True)
    except Exception as error:
        code = 1
        traceback.print_exc()
        if simulation is not None:
            simulation.observer = None
            simulation.save({'passed': False, 'error': str(error)})
        if film is not None:
            film.close()
    finally:
        if simulation is not None:
            simulation.world.stop()
        app.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
