^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2020-03-20)
------------------
* Updated install targets and added OpenSSL to dependencies.
* Added support for lookups from devel spaces.
* Changed action client specialization default values for queue sizes to prevent information loss.
* Added missing typedefs.
* Added specialization for action client to allow creating an ActionClient with a BabelFishAction.
* Renamed size and data to more descriptive names _sizeInBytes and _stream.
  size and data will be removed in future release.
* Configuration for Travis and CodeCov. Added test depends. Set CMake min to 3.0 which should work with kinetic but due to the tests requiring melodic, kinetic will remain officially unsupported even though it should work.
* Removed embedded python description provider. Improved description provider performance.
* Moved message from stream creation to static fromStream method.
  CompoundMessage now uses template to auto-initialize for easier message construction.
  Changed ArrayMessage interface to use vector-like interface. Old methods deprecated and will be removed in a future version.
* Renamed targets to include ${PROJECT_NAME} prefix to fix `#3 <https://github.com/StefanFabian/ros_babel_fish/issues/3>`_.
* Moved benchmarks to separate repo.
* Added message extractor.
  Updated benchmarks with results for message extractor.
  Fixed examples using old array structure.
* Added integrated description provider.
  Added deprecated built-ins byte and char.
  Added benchmark.
* More error checking for embedded python.
* Updated description for initial release.
* Commented action stuff which is not yet ready for release.
* Merge branch 'master' of github.com:StefanFabian/ros_babel_fish
* Cleanup for initial release.
  Added convenience methods.
  Added a lot of tests.
* Cleanup for initial release.
  Added convenience methods.
  Added a lot of tests.
* Added error handling for unknown service. Added special CompoundArrayMessage subclass to provide the element's datatype.
* Added bool as explicit type.
* Initial commit
* Contributors: Stefan Fabian
