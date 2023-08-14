import ament_index_python
from rosidl_cli.command.generate.api import generate
from pathlib import Path
import shutil

interfaces_pkgs = ament_index_python.get_resources('rosidl_interfaces')

for pkg_name, pkg_prefix in interfaces_pkgs.items():
    resource_files, rsc_path = ament_index_python.get_resource('rosidl_interfaces', pkg_name)
    resource_files = resource_files.split('\n')

    share_path = ament_index_python.get_package_share_path(pkg_name).relative_to(Path.cwd())

    idls = [str(share_path / resource_file) for resource_file in resource_files if resource_file.endswith('.idl')]
    print(f'Generating {pkg_name} interfaces from {resource_files}...')

    tmp_path = Path('/tmp/test')
    generate(
        package_name=pkg_name,
        interface_files=idls,
        output_path=tmp_path,
        types=['mypy']
    )

    dest_path = Path(f"{pkg_prefix}/lib/python3.11/site-packages/{pkg_name}")

    # this generates in a weird path: /tmp/test/install/visualization_msgs/share/*
    # move the generated files to the relevant path
    for item in (tmp_path / share_path).iterdir():
        if not item.is_dir():
            continue

        for sub_item in item.iterdir():
            shutil.move(str(sub_item), str(dest_path / item.name / sub_item.name))

    break