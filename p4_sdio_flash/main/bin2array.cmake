# Creates C resources file from files in given directory recursively
function(create_resources dir output)
    file(WRITE ${output} "#include <stdint.h>\n\n")
    file(GLOB bin_paths ${dir}/ESP*/*)
    foreach(bin ${bin_paths})
        file(GLOB name RELATIVE ${dir} ${bin})
        string(REGEX REPLACE "[\\./-]" "_" filename ${name})
        file(READ ${bin} filedata HEX)
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata ${filedata})
        execute_process(COMMAND md5sum ${bin} OUTPUT_VARIABLE md5_output)
        string(REGEX REPLACE " .*" "" md5_hash ${md5_output})
        file(APPEND ${output}
            "const uint8_t  ${filename}[] = {${filedata}};\n"
            "const uint32_t ${filename}_size = sizeof(${filename});\n"
            "const uint8_t  ${filename}_md5[] = \"${md5_hash}\";\n"
        )
    endforeach()
endfunction()

