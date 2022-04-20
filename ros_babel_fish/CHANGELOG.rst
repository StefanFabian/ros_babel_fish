^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_babel_fish
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.3 (2022-04-20)
------------------
* Added some more information to exception messages.
* Add libssl-dev as a build-export dependency
* Contributors: AR Dabbour, Stefan Fabian

0.9.2 (2021-12-22)
------------------
* Better rosbag support + example (`#7 <https://github.com/StefanFabian/ros_babel_fish/issues/7>`_)
  * Accept a more general IBabelFishMessage interface in read-only functions.
  * Add a rosbag example, add some useful method overloads.
* Add noetic to CI, improvements to type safety checking (`#6 <https://github.com/StefanFabian/ros_babel_fish/issues/6>`_)
  * Add noetic to CI.
  * Simplify isCompatible() and inBounds() checks, add tests.
  * Moved compatibility checks to header and internal namespace.
  Co-authored-by: Stefan Fabian <fabian@sim.tu-darmstadt.de>
* Accept True and False as boolean constants (`#4 <https://github.com/StefanFabian/ros_babel_fish/issues/4>`_)
  Co-authored-by: Stefan Fabian <fabian@sim.tu-darmstadt.de>
* Added more explicit warning if message type is not a valid message name.
* Made BabelFishMessageException subclass of BabelFishException.
* Contributors: Martin Valgur, Stefan Fabian

0.9.1 (2020-10-29)
------------------
* Added missing build depend for openssl headers to fix build for noetic.
* Contributors: Stefan Fabian

0.9.0 (2020-10-28)
------------------
* Added install target for service_info. Cleaned up package.xml
* Fixed service spec loading wrong response if service spec doesn't contain '---'.
* Updated integrated description provider for service descriptions to reflect changes in genmsg that caused some tests to fail.
  Both implementations are interoperable and the change should have no impact on service calls using ros_babel_fish but new implementation is now again consistent with message_traits::definition<...>().
  The new implementation is also less complex and faster.
* Added macros for template calls.
  This replaces the boiler plate code for situations where you would do switch(msg.type()) { ... } and call a template function with the C++ type of the message as a template parameter.
* Contributors: Stefan Fabian

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
