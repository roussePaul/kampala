Kampala package
===============

Metapackage used to gather every package in one folder that just need to be add inside the src directory of the catkin workspace to work.

# Usage
To add a package just add a run_depend tag with the name of the package in the [package.xml](package.xml) file.
```XML
<run_depend>my_new_package</run_depend>
```