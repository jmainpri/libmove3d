varying vec3 normal, eyeVec;

void main()
{
	float intensity;
  vec3 eyeVec_n;
	vec4 final_color;
	
	// normalizing the lights position to be on the safe side
	vec3 n = normalize(normal);
	
  float x, x1, x2, x3, x4, x5;

  eyeVec_n= normalize(eyeVec);
  x= dot(eyeVec_n,n);

  if(x < 0.0)
  { x= 0.0; }

  if(x > 1.0)
  { x= 1.0; }


  x1= 1.0/6.0;
  x2= 2.0/6.0;
  x3= 0.5;
  x4= 4.0/6.0;
  x5= 5.0/6.0;

  if(x < x1)
  {
    final_color[0]= 1.0;
    final_color[1]= x/x1;
    final_color[2]= 0.0;
  }
  else if(x < x2)
  {
    final_color[0]= (x2-x)/(x2-x1);
    final_color[1]= 1.0;
    final_color[2]= 0.0;
  }
  else if(x < x3)
  {
    final_color[0]= 0.0;
    final_color[1]= 1.0;
    final_color[2]= (x-x2)/(x3-x2);
  }
  else
  {
    final_color[0]= 0.0;
    final_color[1]= (1.0-x)/(1.0-x3);
    final_color[2]= 1.0;
  }

	final_color[3]= 1.0;
	gl_FragColor = final_color;
} 
