
from parameters.parameters import ParametersConfigurator

from dependency_injector import containers, providers
from pathlib import Path


class GlobalContainer(containers.DeclarativeContainer):

    parametes_updater = providers.Singleton(
        ParametersConfigurator,
        str(Path.home()) + "/05_rotor_mkt/src/http_core/param/dynamic_params.yml"
    )
