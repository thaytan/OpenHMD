project(
	'openhmd', 'c', 'cpp',
	default_options: 'c_std=c99',
	version: '0.3.0',
	meson_version: '>= 0.44',
)
library_version = '0.1.0'


#
# Dependencies
#

hidapi = 'hidapi'
_hidapi = get_option('hidapi')
if host_machine.system() == 'linux'
	if _hidapi == 'hidraw'
		hidapi = 'hidapi-hidraw'
	else
		hidapi = 'hidapi-libusb'
	endif
else
	if _hidapi != 'auto'
		warning('hidapi option ignored on non-Linux systems')
	endif
endif

dep_libm = meson.get_compiler('c').find_library('m', required: false)
dep_hidapi = dependency(hidapi, required : false)
if not dep_hidapi.found()
	proj_hidapi = subproject('hidapi')
	dep_hidapi = proj_hidapi.get_variable('hidapi_dep')
endif
dep_threads = dependency('threads')
dep_libusb = dependency('libusb-1.0', required: false)
if not dep_libusb.found()
	proj_hidapi = subproject('hidapi')
	dep_libusb = proj_hidapi.get_variable('libusb_dep')
endif
dep_threads = dependency('threads')
dep_opencv = dependency('opencv4', required: false)
if not dep_opencv.found()
	# On Windows, search via cmake
        dep_opencv = dependency('opencv', method: 'cmake', modules: ['opencv_core'])
endif
if not dep_opencv.found()
	dep_opencv = dependency('opencv')
endif

deps = [
	dep_libm,
	dep_threads,
]


#
# Main lib
#

sources = [
	'src/openhmd.c',
	'src/drv_dummy/dummy.c',
	'src/omath.c',
	'src/exponential-filter.c',
	'src/fusion.c',
	'src/matrices.c',
	'src/shaders.c',
	'src/unscented.c',
	'src/ukf.c',
]
if host_machine.system() == 'windows'
	sources += 'src/platform-win32.c'
else
	sources += 'src/platform-posix.c'
endif

c_args = []
debugging_mode = get_option('debugging_mode')
have_pipewire = false
if debugging_mode
	c_args += '-DOHMD_DEBUGGING_MODE'

	pw_dep = dependency('libpipewire-0.3', version : '>= 0.3.0', required : false)
	spa_dep = dependency('libspa-0.2', version : '>= 0.2', required: false)
	have_pipewire = debugging_mode and pw_dep.found() and spa_dep.found()
endif

if have_pipewire
	c_args += '-DHAVE_PIPEWIRE=1'
	c_args += '-DHAVE_DEBUG_STREAM=1'
	deps += [ pw_dep, spa_dep ]
	sources += [ 'src/drv_oculus_rift/ohmd-pipewire.c' ]
endif

have_gstreamer = false
if debugging_mode
  gst_dep = dependency('gstreamer-1.0', fallback : ['gstreamer', 'gst_dep'], required: false)
  gstvideo_dep = dependency('gstreamer-video-1.0', fallback: ['gst-plugins-base', 'video_dep'], required: false)
  gstapp_dep = dependency('gstreamer-app-1.0', fallback: ['gst-plugins-base', 'app_dep'], required: false)

  have_gstreamer = gst_dep.found() and gstvideo_dep.found() and gstapp_dep.found()
  if have_gstreamer
	  c_args += '-DHAVE_GSTREAMER=1'
	  deps += [ gst_dep, gstvideo_dep, gstapp_dep ]
		sources += ['src/drv_oculus_rift/ohmd-gstreamer.c']
  endif
endif

if get_option('default_library') == 'shared'
	if host_machine.system() == 'windows'
		c_args += '-DDLL_EXPORT'
	else
		c_args += '-fvisibility=hidden'
	endif
else
	if host_machine.system() == 'windows'
		c_args += '-DOHMD_STATIC'
	endif
endif

_drivers = get_option('drivers')
if _drivers.contains('rift')
	sources += [
		'src/drv_oculus_rift/rift.c',
		'src/drv_oculus_rift/rift-hmd-radio.c',
		'src/drv_oculus_rift/packet.c',
		'src/drv_oculus_rift/rift-sensor.c',
		'src/drv_oculus_rift/rift-sensor-opencv.cpp',
		'src/drv_oculus_rift/rift-sensor-maths.c',
		'src/drv_oculus_rift/rift-sensor-ar0134.c',
		'src/drv_oculus_rift/rift-sensor-mt9v034.c',
		'src/drv_oculus_rift/rift-sensor-blobwatch.c',
		'src/drv_oculus_rift/rift-sensor-esp570.c',
		'src/drv_oculus_rift/rift-sensor-esp770u.c',
		'src/drv_oculus_rift/rift-sensor-flicker.c',
		'src/drv_oculus_rift/rift-sensor-pose-helper.c',
		'src/drv_oculus_rift/rift-sensor-uvc.c',
		'src/drv_oculus_rift/rift-tracker.c',
		'src/drv_oculus_rift/rift-kalman-6dof.c',
    'src/drv_oculus_rift/correspondence_search.c',
    'src/drv_oculus_rift/led_search.c',
		'src/ext_deps/nxjson.c'
	]
	c_args += '-DDRIVER_OCULUS_RIFT'
	c_args += '-DHAVE_OPENCV=1'
	deps += dep_hidapi
	deps += dep_libusb
	deps += dep_opencv
  if have_pipewire
		sources += ['src/drv_oculus_rift/rift-debug-draw.c']
  endif
endif

_drivers = get_option('drivers')
if _drivers.contains('rift-s')
	sources += [
		'src/drv_oculus_rift_s/rift-s.c',
		'src/drv_oculus_rift_s/rift-s-controller.c',
		'src/drv_oculus_rift_s/rift-s-protocol.c',
		'src/drv_oculus_rift_s/rift-s-firmware.c',
		'src/drv_oculus_rift_s/rift-s-radio.c',
		'src/ext_deps/nxjson.c',
	]
	c_args += '-DDRIVER_OCULUS_RIFT_S'
	deps += dep_hidapi
endif

if _drivers.contains('deepoon')
	sources += [
		'src/drv_deepoon/deepoon.c',
		'src/drv_deepoon/packet.c',
	]
	c_args += '-DDRIVER_DEEPOON'
endif

if _drivers.contains('psvr')
	sources += [
		'src/drv_psvr/psvr.c',
		'src/drv_psvr/packet.c',
	]
	c_args += '-DDRIVER_PSVR'
	deps += dep_hidapi
endif

if _drivers.contains('vive')
	sources += [
		'src/drv_htc_vive/vive.c',
		'src/drv_htc_vive/packet.c',
		'src/ext_deps/nxjson.c',
	]
	c_args += '-DDRIVER_HTC_VIVE'
	deps += dep_hidapi
endif

if _drivers.contains('nolo')
	sources += [
		'src/drv_nolo/nolo.c',
		'src/drv_nolo/packet.c',
	]
	c_args += '-DDRIVER_NOLO'
	deps += dep_hidapi
endif

if _drivers.contains('wmr')
	sources += [
		'src/drv_wmr/wmr.c',
		'src/drv_wmr/packet.c',
		'src/ext_deps/nxjson.c'
	]
	c_args += '-DDRIVER_WMR'
	deps += dep_hidapi
endif

if _drivers.contains('xgvr')
	sources += [
		'src/drv_3glasses/xgvr.c',
		'src/drv_3glasses/packet.c',
	]
	c_args += '-DDRIVER_XGVR'
	deps += dep_hidapi
endif

if _drivers.contains('vrtek')
	sources += [
		'src/drv_vrtek/vrtek.c',
		'src/drv_vrtek/packet.c',
	]
	c_args += '-DDRIVER_VRTEK'
	deps += dep_hidapi
endif

if _drivers.contains('external')
	sources += [
		'src/drv_external/external.c',
	]
	c_args += '-DDRIVER_EXTERNAL'
endif

if _drivers.contains('android')
	sources += [
		'src/drv_android/android.c',
	]
	c_args += '-DDRIVER_ANDROID'
endif

openhmd_lib = library(
	'openhmd',
	sources,
	include_directories: include_directories('./include'),
	c_args: c_args,
	cpp_args: c_args,
	dependencies: deps,
	install: true,
	version: library_version,
)


#
# Examples
#

_examples = get_option('examples')

# Simple
if _examples.contains('simple')
	deps = [dep_threads]

	simple_sources = [
		'examples/simple/simple.c',
	]

	executable(
		'openhmd_simple_example',
		simple_sources,
		include_directories: include_directories('./include'),
		link_with: [openhmd_lib],
		dependencies: deps,
		install: true,
	)
endif

# OpenGL
if _examples.contains('opengl')

	dep_sdl2 = dependency('sdl2')
	dep_gl = dependency('gl')
	dep_glew = dependency('glew')
	deps = [dep_sdl2, dep_gl, dep_glew, dep_threads]

	opengl_sources = [
		'examples/opengl/main.c',
		'examples/opengl/gl.c',
	]

	executable(
		'openhmd_opengl_example',
		opengl_sources,
		include_directories: include_directories([
			'./include',
			'examples/opengl'
		]),
		link_with: [openhmd_lib],
		dependencies: deps,
		install: true,
	)
endif


#
# Install and pkg-config export file
#

desc = 'API and drivers for immersive technology devices such as HMDs'

pkg = import('pkgconfig')
pkg.generate(
	name: 'openhmd',
	description: desc,
	version: meson.project_version(),
	subdirs: 'openhmd',
	requires: hidapi,
	libraries: openhmd_lib,
	url: 'http://www.openhmd.net/',
)
install_headers('include/openhmd.h', subdir: 'openhmd')

# Declare openhmd as a dependency so it can be used
# as a meson subproject
openhmd_dep = declare_dependency(
  include_directories: include_directories('./include'),
  link_with : openhmd_lib)

#
# Unit tests
#

if get_option('tests')
	unittests_sources = [
		'src/omath.c',
		'tests/unittests/highlevel.c',
		'tests/unittests/main.c',
		'tests/unittests/mat4x4.c',
		'tests/unittests/quat.c',
		'tests/unittests/tests.h',
		'tests/unittests/vec.c',
		'tests/unittests/pose.c'
	]

	unittests = executable(
		'openhmd_unittests',
		unittests_sources,
		include_directories: include_directories('./include', './src'),
		link_with: [openhmd_lib],
		dependencies: [dep_libm, dep_threads]
	)

	test('unittests', unittests)
endif
