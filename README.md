# Example application using a LoRa module, using `asimple`

This is an example application using the asimple and forked AmbiqSuiteSDK
projects to sense the voltage of an attached LMT84 and send it as JSON over
LoRa PHY.

## Dependencies
 - https://github.com/gemarcano/AmbiqSuiteSDK
 - https://github.com/gemarcano/asimple

In order for the libraries to be found, `pkgconf` must know where they are. The
special meson cross-file property `sys_root` is used for this, and the
`artemis` cross-file already has a shortcut for it-- it just needs a
variable to be overriden. To override a cross-file constant, you only need to
provide a second cross-file with that variable overriden. For example:

Contents of `my_cross`:
```
[constants]
prefix = '/home/gabriel/.local/redboard'
```

# Compiling and installing
```
mkdir build
cd build
# The `artemis` cross-file is assumed to be installed per recommendations from
# the `asimple` repository
meson setup --prefix [prefix-where-sdk-installed] --cross-file artemis --cross-file ../my_cross --buildtype release
meson install
```

# License

See the license file for details. In summary, this project is licensed
Apache-2.0, except for the bits copied from the Ambiq SDK.
