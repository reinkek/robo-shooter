FILE(REMOVE_RECURSE
  "CMakeFiles/exampleCode.dir/src/exampleCode.o"
  "../bin/exampleCode.pdb"
  "../bin/exampleCode"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/exampleCode.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
