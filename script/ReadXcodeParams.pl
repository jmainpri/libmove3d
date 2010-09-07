#!/usr/bin/perl

# This perl script reads the input xcode pbxuser file and puts execution preferences into a temp file

open(INPUT,"<$ARGV[0]") or die "Usage ReadXcodeParams .pbxuser output\n";
@file_contents=<INPUT>;
close(INPUT);

open(OUTPUT,">$ARGV[1]") or die "Usage ReadXcodeParams .pbxuser output\n";

$file_in_string=join("",@file_contents);

if( $file_in_string =~ /activeArgIndices(.*?)\;/ims ){
  print(OUTPUT "activeArgIndices$1;\n");
}

if( $file_in_string =~ /argumentStrings(.*?)\;/ims ){
  print(OUTPUT "argumentStrings$1;\n");
}

if( $file_in_string =~ /environmentEntries(.*?)\);/ims ){
  print(OUTPUT "environmentEntries$1);\n");
}

close(OUTPUT);
