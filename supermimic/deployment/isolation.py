"""Create loopback-only simulation networks; never relax the namespace check."""
import os
import shutil
import subprocess

SETUP = 'ip link set lo up || exit 1; exec "$@"'

def namespace_prefix():
    if os.geteuid() == 0:
        choices = [('root_network_namespace', ['unshare', '-n', 'bash', '-c', SETUP, 'isolated-supermimic'])]
    else:
        choices = [('user_network_namespace', ['unshare', '-Urn', 'bash', '-c', SETUP, 'isolated-supermimic'])]
        # Match the colleague's sudo + network namespace approach. Drop back
        # to the calling uid before running any Python simulation/controller.
        if shutil.which('sudo') and shutil.which('setpriv'):
            choices.append(('sudo_network_namespace_drop_uid',
                ['sudo', '-n', 'unshare', '-n', 'bash', '-c', SETUP, 'isolated-supermimic',
                 'setpriv', '--reuid', str(os.getuid()), '--regid', str(os.getgid()), '--init-groups', '--']))
    errors=[]
    for mode, prefix in choices:
        p=subprocess.run([*prefix, '/usr/bin/true'],capture_output=True,text=True,timeout=15)
        if p.returncode == 0:
            return mode, prefix
        errors.append(mode + ': ' + p.stderr.strip())
    raise RuntimeError('Cannot create isolated simulation network. If sudo needs authentication, run sudo -v locally and retry. No robot commands sent. ' + '; '.join(errors))

def isolated_command(argv, env):
    mode, prefix=namespace_prefix()
    # sudo filters the calling environment: restore only runtime settings after
    # leaving root. The simulation opens DDS only after its own interface check.
    allowed=['CYCLONEDDS_HOME','PYTHONPATH','LD_LIBRARY_PATH','OMP_NUM_THREADS',
             'OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','PYTHONDONTWRITEBYTECODE']
    settings=[k+'='+env[k] for k in allowed if k in env]
    return mode, [*prefix, '/usr/bin/env', *settings, *argv]
