import os
from glob import glob
from setuptools import find_packages, setup
from torch.utils import cpp_extension

# python3 setup.py build_ext --inplace --verbose

SEN_ALGO_BASE_ROOT = 'SensingAlgoBase'
SEN_ALGO_BASE_NAME = 'sensingalgobase'
MULTI_CAM_REG_NAME = 'multicamregistration'

MKT_VIS_ONNX_ROOT = 'Market-vision-restocking_dep'
MKT_MKT_RSTK_NAME = 'MarketRestockingDep'

SEN_ALGO_BASE = f'{SEN_ALGO_BASE_ROOT}/{SEN_ALGO_BASE_NAME}'
MULTI_CAM_REG = f'{SEN_ALGO_BASE_ROOT}/{MULTI_CAM_REG_NAME}'

MKT_VIS = f'{MKT_VIS_ONNX_ROOT}/{MKT_MKT_RSTK_NAME}'

package_name = 'vision_recognition'

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    ext_modules=[        
        cpp_extension.CUDAExtension(
            name='multicamregistration_cpp', 
            sources=[
                f'{SEN_ALGO_BASE_ROOT}/{MULTI_CAM_REG_NAME}/MultiCamRegistration.cpp', 
                f'{SEN_ALGO_BASE_ROOT}/{MULTI_CAM_REG_NAME}/MultiCamRegistration_cuda.cu', 
            ],
            extra_compile_args={'cxx': ['-g', '-std=c++17']},
        ),
    ],
    cmdclass={'build_ext': cpp_extension.BuildExtension},

    data_files=package_files(
        data_files, [
                f'{SEN_ALGO_BASE}/', 
                f'{MULTI_CAM_REG}/', 
                f'{MKT_VIS}/', 
                'config'
            ]
        ),
    install_requires=[
        'setuptools==58.2.0',
        'torch',
        ],
    zip_safe=True,
    maintainer='mskwok',
    maintainer_email='mskwok@hkclr.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognition_server = vision_recognition.recognition_server:main',
            'fake_recognition_server = vision_recognition.fake_recognition_server:main',
            'calibration_helper = vision_recognition.calibration_helper:main'
        ],
    },
)
