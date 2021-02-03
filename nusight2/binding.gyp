{
    'targets': [
        {
            'target_name': 'nbs_player',
            'sources': [
                'src/server/nbs/mmap_nbs_player/nbs_player.cpp',
                'src/server/nbs/mmap_nbs_player/nbs_play_action.cpp',
            ],
            'cflags': [
            ],
            'include_dirs': [
                '<!(node -e "require(\'nan\')")',
            ],
            'msvs_settings': {
                'VCCLCompilerTool': {
                    'ExceptionHandling': 1
                }
            },
            'conditions': [
                [
                    'OS=="linux"', {
                        'ccflags': [
                            '-std=c++14'
                            '-fPIC',
                            '-fext-numeric-literals',
                            '-fexceptions'
                        ],
                        'ccflags!': [
                            '-fno-exceptions'
                        ],
                        'cflags_cc': [
                            '-std=c++14',
                            '-fext-numeric-literals'
                        ],
                        'cflags_cc!': [
                            '-fno-exceptions',
                            '-fno-rtti'
                        ],
                    }
                ],
                [
                    'OS=="mac"', {
                        'ccflags': [
                            '-std=c++14'
                            '-fPIC',
                            '-fext-numeric-literals',
                            '-fexceptions'
                        ],
                        'ccflags!': [
                            '-fno-exceptions'
                        ],
                        'cflags_cc': [
                            '-std=c++14',
                            '-fext-numeric-literals'
                        ],
                        'cflags_cc!': [
                            '-fno-exceptions',
                            '-fno-rtti'
                        ],
                        'xcode_settings': {
                            'MACOSX_DEPLOYMENT_TARGET': '10.9',
                            'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
                            'GCC_ENABLE_CPP_RTTI': 'YES',
                            'OTHER_CPLUSPLUSFLAGS': ['-std=c++14', '-stdlib=libc++'],
                            'OTHER_LDFLAGS': ['-stdlib=libc++']
                        }
                    }
                ]
            ]
        }
    ]
}
