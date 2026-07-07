from setuptools import setup

VERSION = '2.2.1'

setup(
    name='pyvesc',
    packages=['pyvesc', 'pyvesc.protocol', 'pyvesc.protocol.packet', 'pyvesc.messages',
              'pyvesc.params', 'pyvesc.can'],
    version=VERSION,
    python_requires='>=3.7',
    description='VESC communication protocol in Python — Molten MOSFET dyno fork '
                '(serial/TCP COMM protocol + native CAN runtime layer, four-quadrant '
                'command set, firmware flashing).',
    author='Molten MOSFET (fork of Liam Bindle\'s PyVESC)',
    author_email='jotham@wearebasis.com',
    url='https://github.com/moltenmosfet/PyVESC',
    download_url='https://github.com/moltenmosfet/PyVESC/tarball/' + VERSION,
    keywords=['vesc', 'VESC', 'communication', 'protocol', 'packet', 'dyno', 'can',
              'socketcan'],
    classifiers=[],
    install_requires=['crccheck'],
    extras_require={
        'can': ['python-can>=4'],      # native CAN runtime (pyvesc.can.bus/node)
        'serial': ['pyserial'],        # COMM protocol over serial
    },
)
