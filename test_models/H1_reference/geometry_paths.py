"""One explicit geometry family for simulation, replay and independent CAD audit."""
from pathlib import Path


def mesh_directory(config):
    path=Path(config['_root'])/config['collision']['mesh_directory']
    if not path.is_dir():raise FileNotFoundError(path)
    return path


def cad_directory(config):
    path=mesh_directory(config).parent/'cad'
    if not (path/'H1_A_temporary_assembly.step').is_file():raise FileNotFoundError(path)
    return path
