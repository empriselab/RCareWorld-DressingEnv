#### To use this environment with RCareWorld:

1. Make sure you have the scene files from the [dressing repo](https://github.com/empriselab/dressing_simulation). 
    1. Under the python-api branch in the Hospital Bed directory is a file called Test Robot with Bed.unity
2. The dressing_env.py should be put into under /rcare_py/pyrcareworld/pyrcareworld/envs
    1. The init.py file should be modified to import the dressing environment with
        > from .dressing_env import DressingEnv
3. Replace the rcareworld_env.py file in your RCareworld install with what the rcareworld_env.py file here. 
4. Place the file device.py under the directory rcare_py/pyrcareworld/pyrcareworld/objects
    1. The init.py file should be modified to import the bed object with
        > from .device import RCareWorldBed

5. Use test_dressing.py with the scene to test the environment. 
 
