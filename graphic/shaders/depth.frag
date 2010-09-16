

void main()
{
	vec4 final_color;
	
  final_color[0]= gl_FragCoord.z*gl_FragCoord.w;
  final_color[1]= gl_FragCoord.z*gl_FragCoord.w;
  final_color[2]= gl_FragCoord.z*gl_FragCoord.w;
	final_color[3]= 1.0;

	gl_FragColor = final_color;
} 
