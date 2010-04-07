# This perl script reads the input xcode pbxuser file and replaces the execution preferences from a temp file

open(INPUT,"<$ARGV[0]") or exit;
@temp_file_contents=<INPUT>;
close(INPUT);

unlink("$ARGV[0]");

open(OUTPUT,"<$ARGV[1]") or exit;
@xcode_file_contents=<OUTPUT>;
close(OUTPUT);

$temp_file_in_string=join("",@temp_file_contents);
$xcode_file_in_string=join("",@xcode_file_contents);

if( $temp_file_in_string =~ /activeArgIndices(.*?)\;/ims ){
#  print($1);
  $saved_data = "activeArgIndices" . $1 . ";" ;
  $xcode_file_in_string =~ s/activeArgIndices(.*?)\;/$saved_data/mgs;
}

if( $temp_file_in_string =~ /argumentStrings(.*?)\;/ims ){
#  printf($1);
  $saved_data = "argumentStrings" . $1 . ";" ;
  $xcode_file_in_string =~ s/argumentStrings(.*?)\;/$saved_data/mgs;
}

#print "$xcode_file_in_string" ;

open(OUTPUT,">$ARGV[1]") or die;
print OUTPUT $xcode_file_in_string ;
close(OUTPUT);

print "Xcode settings restored\n"
