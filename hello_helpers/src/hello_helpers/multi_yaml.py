from launch import Substitution, LaunchContext, SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from collections.abc import Mapping
import tempfile
from typing import List, Text
import yaml


def recursive_update(d, u):
    for k, v in u.items():
        if isinstance(v, Mapping):
            d[k] = recursive_update(d.get(k, {}), v)
        else:
            d[k] = v
    return d


class MultiYaml(Substitution):
    def __init__(
        self,
        source_files: List[SomeSubstitutionsType],
    ) -> None:
        super().__init__()

        self.__source_files = source_files

    def perform(self, context: LaunchContext) -> Text:
        output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)

        d = {}

        for raw_source_file in self.__source_files:
            source_list = normalize_to_list_of_substitutions(raw_source_file)
            source_file = perform_substitutions(context, source_list)

            new_d = yaml.safe_load(open(source_file))
            recursive_update(d, new_d)

        yaml.dump(d, output_file)

        return output_file.name
