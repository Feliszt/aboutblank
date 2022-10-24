#version 150

//
uniform sampler2DRect tex0;
uniform float minX;
uniform float maxX;
uniform float minY;
uniform float maxY;
uniform vec4 colorToDetect;
uniform float colorThresh;

//
in vec2 texCoordVarying;

// this is the output of the fragment shader
out vec4 outputColor;

void main(){
	// get texte
	vec4 texel0 = texture(tex0, texCoordVarying);

	if (texCoordVarying.x < minX || texCoordVarying.x > maxX || texCoordVarying.y < minY || texCoordVarying.y > maxY) {
		outputColor = vec4(0.0, 0.0, 0.0, 1.0);
		return;
	}

	float r1 = texel0.r * 255;
	float g1 = texel0.g * 255;
	float b1 = texel0.b * 255;

	// set Bg and Fg colors
	vec4 colBg = vec4(0.0, 0.0, 0.0, 1.0);
	vec4 colFg = vec4(1.0, 1.0, 1.0, 1.0);

	// copy uniforms
	float r2 = colorToDetect.r;
	float g2 = colorToDetect.g;
	float b2 = colorToDetect.b;

	// get dist
	float r = (r1 + r2) * 0.5;
	float deltaR = r1 - r2;
	float deltaG = g1 - g2;
	float deltaB = b1 - b2;
	float distColor = sqrt((2 + r / 256) * deltaR * deltaR + 4 * deltaG * deltaG + (2 + (256 - r) / 256) * deltaB * deltaB);
	distColor = distColor / 675;

	// threshold
	vec4 c = vec4(0.0, 0.0, 0.0, 1.0);
	if(distColor <= colorThresh) {
		c = vec4(1.0, 1.0, 1.0, 1.0);
	}

	outputColor = c;
}