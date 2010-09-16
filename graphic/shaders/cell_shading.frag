varying vec3 lightDir, normal;
varying vec4 color;

void main()
{
	float intensity;
  vec3 lightDir_n;
	vec4 final_color;
	
	// normalizing the lights position to be on the safe side
	
	vec3 n = normalize(normal);
	
  lightDir_n= normalize(lightDir);
	intensity = dot(lightDir_n, n);


	if (intensity > 0.66)
		final_color = 0.95*vec4(color[0], color[1], color[2], 1.0);
	else if (intensity > 0.33)
		final_color = 0.5*vec4(color[0], color[1], color[2], 1.0);
	else
		final_color = 0.1*vec4(color[0], color[1], color[2], 1.0);

	/*
	if (intensity > 0.95)
		final_color = 1.0*vec4(color[0], color[1], color[2], 1.0);
	else if (intensity > 0.5)
		final_color = 0.6*vec4(color[0], color[1], color[2], 1.0);
	else if (intensity > 0.25)
		final_color = 0.4*vec4(color[0], color[1], color[2], 1.0);
	else
		final_color = 0.2*vec4(color[0], color[1], color[2], 1.0);
*/


	final_color[3]= 1.0;
	gl_FragColor = final_color;
} 
