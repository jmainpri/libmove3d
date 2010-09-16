// Programmed by: Kip Ricker

uniform sampler2D colorMap;

//#define width
#define offset 1.0/512.0

mat3 Gx = mat3(1.0,2.0,1.0, 0.0,0.0,0.0, -1.0,-2.0,-1.0);
mat3 Gy = mat3(1.0,0.0,-1.0, 2.0,0.0,-2.0, 1.0,0.0,-1.0);

vec4 sobel( sampler2D tex, vec2 uv )
{
	float i;
	float j;
	
	float sumX;
	float sumY;

	float bw;
	vec4 tmp;

	//Soble calculation
	for(i=-1.0; i<=1.0; ++i){
		for(j=-1.0; j<=1.0; ++j){
			tmp = texture2D( colorMap, gl_TexCoord[0].xy + vec2(offset*i,offset*-j) );
			bw = (tmp.r+tmp.g+tmp.b)/3.0;
			sumX = sumX + bw * Gx[int(i+1.0)][int(j+1.0)];
			sumY = sumY + bw * Gy[int(i+1.0)][int(j+1.0)];
		}
	}
	
	bw = abs(sumX) + abs(sumY);
	
	//This makes the edges thicker	
	bw = bw*2.0;

	//Clamp black and white value
	if( bw > 1.0 ){
		bw = 1.0;
	}

	//Invert sobel colors
	// Without this, you get a neat effect that just draws
	// the sobel edges with the original colors on black.
	bw = 1.0-bw;

	//Show just the soble edges that are darker then 20%
	// to draw just the black lines.
	if( bw < 0.2 ){
		tmp = vec4(bw,bw,bw,1.0);
	}



	return tmp;
}

void main (void)
{
	gl_FragColor = sobel( colorMap, gl_TexCoord[0].xy );
}