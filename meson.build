project('rba_lora', 'c',
  default_options: [
    'b_lto=true',
    'b_staticpic=false',
    'c_std=c2x',
    'warning_level=3',
  ],
  license: ['Apache-2.0'],
  version: '0.1.0')

c_args = [
  '-ffunction-sections',
]

link_args = [
  '-Wl,--gc-sections', '-fno-exceptions',
]

# Executables -- FIXME this is here more as an example for now...
sources = files([
  'src/main.c',
])

# Find libm...
cc = meson.get_compiler('c', native: false)
m_dep = cc.find_library('m', required : false)

ambiq_lib = dependency('ambiq_rba_atp')
asimple_lib = dependency('asimple_rba_atp')
exe = executable(meson.project_name(),
  sources,
  dependencies: [ambiq_lib, m_dep, asimple_lib],
  c_args: c_args,
  link_args: link_args + ['-T' + meson.source_root() / 'linker.ld']
)

# Custom flash target, to make it easier to program the RedBoard, requires
# objcopy
objcopy = find_program('objcopy')

bin = custom_target(
  input : exe,
  output : exe.name().split('.')[0] + '.bin',
  command : [objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@', ],
  build_by_default: true
)

run_target('flash',
  command : ['python3', meson.source_root() / 'svl.py',
    get_option('tty'), '-f',  bin, '-b', '921600', '-v'],
  depends : bin,
)
