SystemConfiguration
===================

## Description
This module is used to ensure that the configuration that is running on a device matches what is required.

It goes through several steps to ensure the system is setup correctly

### Filesystem
The system will look through each file in the `system/default` directory.
For each of these files it will see if there is an equivalent file in the `system/$HOSTNAME` and use that, otherwise using the default file.
Using this file it will check that the same file in the root directory is identical to this one, and if not it will copy it across.
This will ensure that all the system configuration files match the files that are in the repository.

> ***NOTE***
> The system will only look at files that exist in the `system/default` directory.
> If you have a file that will override for a specific system, you must provide a default file as well.

After this, it will go through the list of permissions changes that are in the `SystemConfiguration.yaml` file and apply them.
This configuration file must list the total permissions as an octal number.

### Packages
After this the system will then attempt to update and upgrade the system using the package manager.
Once this is done it will go through the list of packages in configuration and ensure that they are installed.

### Systemd
The system will then go through the list of configuration entries for systems and ensure that they are enabled.

## Usage
Run the system_configuration role as a superuser and this role will ensure that the system configuration on the platform matches the configuration in the repository.
