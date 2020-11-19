project('gst-openhmd-plugin', 'c', 'cpp', version : '1.0.0', license : 'LGPL')

plugins_install_dir = join_paths(get_option('libdir'), 'gstreamer-1.0')

cc = meson.get_compiler('c')

gst_version = meson.project_version()

api_version = '1.0'

plugin_c_args = ['-DHAVE_CONFIG_H']

cdata = configuration_data()
cdata.set_quoted('PACKAGE_VERSION', gst_version)
cdata.set_quoted('PACKAGE', 'gst-openhmd-plugin')
cdata.set_quoted('GST_LICENSE', 'LGPL')
cdata.set_quoted('GST_API_VERSION', api_version)
cdata.set_quoted('GST_PACKAGE_NAME', 'GStreamer OpenHMD experiment Plug-ins')
cdata.set_quoted('GST_PACKAGE_ORIGIN', 'https://www.openhmd.net')
configure_file(output : 'config.h', configuration : cdata)

dep_libm = meson.get_compiler('c').find_library('m', required: false)
dep_opencv = dependency('opencv')

gst_dep = dependency('gstreamer-1.0',
    fallback : ['gstreamer', 'gst_dep'])

gstvideo_dep = dependency('gstreamer-video-1.0',
    fallback: ['gst-plugins-base', 'video_dep'])

plugin_sources = [
  'gstohmdriftsensor.c',
  '../src/omath.c',
  '../src/drv_oculus_rift/rift-sensor-blobwatch.c',
  '../src/drv_oculus_rift/rift-sensor-flicker.c',
  '../src/drv_oculus_rift/rift-sensor-maths.c',
  '../src/drv_oculus_rift/rift-sensor-opencv.cpp',
  '../src/drv_oculus_rift/rift-sensor-pose-helper.c',
  '../src/drv_oculus_rift/correspondence_search.c',
  '../src/drv_oculus_rift/led_search.c',
  'rift.c'
  ]

c_args = [ plugin_c_args, '-DHAVE_OPENCV=1' ]
add_global_arguments(['-mavx2', '-mfma', '-fno-math-errno'], language : 'c')

gstohmdplugin = library('gstohmdplugin',
  plugin_sources,
  c_args: c_args, cpp_args: c_args,
  include_directories: [ include_directories ('../include'), '../src/drv_oculus_rift' ],
  dependencies : [dep_libm, gst_dep, gstvideo_dep, dep_opencv ],
  install : true,
  install_dir : plugins_install_dir,
)

