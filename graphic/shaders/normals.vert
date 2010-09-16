
varying vec3 normal, eyeVec;
varying vec4 color;


void main()
{
	normal = gl_NormalMatrix * gl_Normal;

	vec3 vVertex = vec3(gl_ModelViewMatrix * gl_Vertex);

	eyeVec = -vVertex;

  gl_FrontColor = gl_Color;
	gl_Position = ftransform();
  color= gl_Color;		
} 
