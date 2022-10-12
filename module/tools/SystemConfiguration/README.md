# SystemConfiguration

## Description

This module is used to ensure that the configuration that is running on a device matches what is required.

It goes through several steps to ensure the system is setup correctly

### Filesystem

The system will look through each file in the `system/default` directory.
For each of these files it will see if there is an equivalent file in the `system/$HOSTNAME` directory and use that, otherwise it uses the default file.
Using this file it will check that the same file in the root directory is identical to this one, and if not it will copy it across.
This will ensure that all the system configuration files match the files that are in the repository.

> **_NOTE_**
> The system will only look at files that exist in the `system/default` directory.
> If you have a file that will override for a specific system, you must provide a default file as well.

After this, it will go through the list of permissions changes that are in the `SystemConfiguration.yaml` file and apply them.
This configuration file must list the total permissions as an octal number.

After permissions are set, it will go through the list of symlinks that are in the `SystemConfiguration.yaml` and ensure they are created.
Checks are made to ensure that the link target exists and the link doesn't exist.

- If the target doesn't exist, no link is made
- If the link name already exists and is a regular file, the file is renamed to `<link name>.old` and the link is made
- If the link name already exists and **is not** a symlink to the target, the link is remove and the correct link is made
- If the link name already exists and **is** a symlink to the target, nothing is done
- If the link name doesn't exist, the link is made

### Packages

After this the system will then attempt to update and upgrade the system using the package manager.
Once this is done it will go through the list of packages in configuration and ensure that they are installed.

### Systemd

The system will then go through the list of configuration entries for systems and ensure that they are enabled.

### Locale

If `generate_locale` is set in the `SystemConfiguration.yaml` then `locale-gen` is called to ensure that system locales have been generated

### Grub

If `generate_grub` is set in the `SystemConfiguration.yaml` then `grub-mkconfig` is called to ensure that the grub configuration file is up to date

### Message Of The Day

The message of the day file is regenerated to ensure it has the correct contents

### Hosts and Hostname

Both the hostname and the hosts file are regenerated with the correct contents

### Groups

The user is then added to all groups listed in the `SystemConfiguration.yaml`, if the user is not currently a member of them

### ZSH

Zprezto is now installed and the appropriate symlinks are made. The users' default shell is also changed to zsh

### Python

Finally, python is updated to ensure that it will search for packages in `/usr/local`

## Usage

Run the system_configuration role as a superuser and this role will ensure that the system configuration on the platform matches the configuration in the repository.
