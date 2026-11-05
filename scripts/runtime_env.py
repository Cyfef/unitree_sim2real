"""Shared subprocess environment for Python simulation and deployment tools."""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def shared_runtime_env() -> dict[str, str]:
    env = dict(os.environ)
    env['CYCLONEDDS_HOME'] = str(ROOT / 'rl_lab_controller/cyclonedds')
    env['PYTHONPATH'] = str(ROOT / 'rl_lab_controller/python_sdk')
    libraries = [ROOT / 'rl_lab_controller/cpp_prefix/lib',
                 ROOT / 'rl_lab_controller/cpp_prefix/usr/lib/x86_64-linux-gnu',
                 ROOT / 'rl_lab_controller/thirdparty/onnxruntime-linux-x64-1.22.0/lib']
    env['LD_LIBRARY_PATH'] = ':'.join(map(str, libraries)) + (':' + os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else '')
    env.update(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    return env
