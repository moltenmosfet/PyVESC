from setuptools import setup

VERSION = '2.0.0'

setup(
    name='pyvesc',
    packages=['pyvesc', 'pyvesc.protocol', 'pyvesc.protocol.packet', 'pyvesc.messages', 'pyvesc.params'],
    version=VERSION,
    python_requires='>=3.6',
    description='VESC communication protocol in Python — Molten MOSFET dyno fork '
                '(serial/TCP, four-quadrant command set, CAN forwarding, firmware flashing).',
    author='Molten MOSFET (fork of Liam Bindle\'s PyVESC)',
    author_email='jotham@wearebasis.com',
    url='https://github.com/moltenmosfet/PyVESC',
    download_url='https://github.com/moltenmosfet/PyVESC/tarball/' + VERSION,
    keywords=['vesc', 'VESC', 'communication', 'protocol', 'packet', 'dyno', 'can'],
    classifiers=[],
    install_requires=['crccheck']
)
