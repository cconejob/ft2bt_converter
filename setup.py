from setuptools import setup, find_packages

# Read the contents of your requirements.txt file
with open('requirements.txt') as f:
    required = f.read().splitlines()

setup(
    name='ft2bt',
    version='0.2.2',
    packages=find_packages(),
    install_requires=required,  # Use the list from requirements.txt
    entry_points={
        'console_scripts': [
            'ft2bt=ft2bt.scripts.ft2bt:main',
        ],
    },
    author='Carlos Conejo',
    author_email='carlos.conejo@upc.edu',
    description='Automatic conversion from fault trees into behavior trees',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/cconejob/ft2bt_converter.git',
    license='GNU GPLv3',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)
