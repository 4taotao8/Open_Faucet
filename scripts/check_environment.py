"""只读检查当前 Python 环境、常用包、GPU 及本地资产，不安装任何东西。"""
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import subprocess
import sys


def main():
    """输出检查结果；缺少必要模块或资产时返回非零退出码。"""
    root = Path(__file__).resolve().parents[1]
    packages = {}
    for distribution, module in [('isaacsim', 'isaacsim'), ('numpy', 'numpy'),
                                  ('scipy', 'scipy'), ('usd-core', 'pxr')]:
        try:
            version = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            version = '由 Isaac Sim 提供' if module == 'pxr' else '未找到'
        packages[module] = {'available': importlib.util.find_spec(module) is not None, 'version': version}
    assets = [root / 'dual_arm/urdf/dual_arm.usd', root / 'single_lever_faucet/single_lever_faucet.usd']
    available_assets = {str(asset.relative_to(root)): asset.is_file() for asset in assets}
    print(json.dumps({'python': sys.executable, 'version': sys.version, 'packages': packages,
                      'assets': available_assets}, ensure_ascii=False, indent=2))
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader'],
                                capture_output=True, text=True, timeout=10)
        print('显卡：' + (result.stdout.strip() or result.stderr.strip()))
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f'显卡检查未完成：{error}')
    return 0 if all(v['available'] for v in packages.values()) and all(available_assets.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
