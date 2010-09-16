
varying vec3 normal, lightDir;
varying vec4 color;


void main()
{
	normal = gl_NormalMatrix * gl_Normal;

	vec3 vVertex = vec3(gl_ModelViewMatrix * gl_Vertex);

	lightDir = vec3(gl_LightSource[0].position.xyz - vVertex);

  gl_FrontColor = gl_Color;
	gl_Position = ftransform();
  color= gl_Color;		
} 
