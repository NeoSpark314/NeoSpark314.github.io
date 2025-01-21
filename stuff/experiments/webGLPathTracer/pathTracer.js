// A Basic Triangle-Based path tracer using WebGL 2.0
// Author: Holger Dammertz (holger.dammertz.org)
// Last edit date: 04.06.2017

//TODOS:
//  - finish phong implementation (eval)
//  - fix/investigate fps counter variance...
//  pack shader in objects
//  material indices are stored per vertex in the triangle texture ... perhaps a second indirection is better?
"use strict";

let canvas, gl;

let shaderCode_Display_vs = `#version 300 es
in vec4 a_position;
out vec2 uv;
void main() {
	uv = (a_position.xy + vec2(1.0)) * 0.5;
	gl_Position = a_position;
}`;

let shaderCode_Display_fs = `#version 300 es
precision highp float;
uniform sampler2D u_sampler2D_ImageToDisplay;
uniform float u_float_InverseFrameNumber;
in vec2 uv;
out vec4 outColor;
void main() {
	vec3 linearColor = texture(u_sampler2D_ImageToDisplay, uv).xyz * u_float_InverseFrameNumber;
	vec3 color = pow(linearColor, vec3(1.0/2.2));
	outColor = vec4(color, 1.0);
}`;

let shaderCode_PathTracer_vs = `#version 300 es

in vec4 a_position;
out vec2 v_PixelCoord;

void main() {
  v_PixelCoord = a_position.xy;
  gl_Position = a_position;
}`;

let shaderCode_PathTracer_fs = `#version 300 es
precision highp float;

in vec2 v_PixelCoord;

uniform vec2 u_vec2_InverseResolution;
uniform mat4 u_mat4_ViewMatrix;
uniform sampler2D u_sampler2D_TriangleData;
uniform sampler2D u_sampler2D_MaterialData;
uniform sampler2D u_sampler2D_BVHData;
uniform int u_int_NumTriangles;

uniform int u_int_FrameNumber;

out vec4 outColor;



// our global RNG state
uvec2 rng_state;
uint george_marsaglia_rng() {
	rng_state.x = 36969u * (rng_state.x & 65535u) + (rng_state.x >> 16u);
	rng_state.y = 18000u * (rng_state.y & 65535u) + (rng_state.y >> 16u);
	return (rng_state.x << 16u) + rng_state.y;
}

float rng_NextFloat() {
	return float(george_marsaglia_rng()) / float(0xFFFFFFFFu);
}

void init_RNG() {
	vec2 offset = vec2(u_int_FrameNumber*17,0.0);

	//Initialize RNG
	rng_state = uvec2(397.6432*(gl_FragCoord.xy+offset));
	rng_state ^= uvec2(32.9875*(gl_FragCoord.yx+offset));
}

const float PI = 3.1415926535897932384626;
const float INV_PI = 1.0 / PI;

vec3 hemisphereSample_cos(float u, float v) {
     float phi = v * 2.0 * PI;
     float cosTheta = sqrt(1.0 - u);
     float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
     return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
 }

// !!TODO: try newer/improved methods here (but check also performance)
void frisvad(const in vec3 n, out vec3 b1, out vec3 b2) {
	if(n.z < -0.9999999) { // Handle the singularity
		b1 = vec3(0.0, -1.0, 0.0);
		b2 = vec3(-1.0, 0.0, 0.0);
		return;
	}
	float a = 1.0 / (1.0 + n.z);
	float b = -n.x*n.y*a;
	b1 = vec3(1.0f - n.x*n.x*a, b, -n.x);
	b2 = vec3(b, 1.0f - n.y*n.y*a, -n.y);
}

mat3 computeONB(const in vec3 n) {
	mat3 ret;
	ret[2] = n;
	frisvad(n, ret[0], ret[1]);
	return ret;
}

struct Ray {
	vec3 direction;
	vec3 origin;
	float tfar;
	vec3 inv_direction;
	ivec3 sign;
};

Ray createRay(in vec3 direction, in vec3 origin, in float tfar) {
    vec3 inv_direction = vec3(1.0) / direction;
	
    return Ray(
        direction,
        origin,
		tfar,
        inv_direction,
		ivec3((inv_direction.x < 0.0) ? 1 : 0,
         (inv_direction.y < 0.0) ? 1 : 0,
         (inv_direction.z < 0.0) ? 1 : 0)
    );
}

bool intersectAABB(const in Ray ray, const in vec3 aabb[2], out float tmin, out float tmax) {
    float tymin, tymax, tzmin, tzmax;
    tmin = (aabb[ray.sign[0]].x - ray.origin.x) * ray.inv_direction.x;
    tmax = (aabb[1-ray.sign[0]].x - ray.origin.x) * ray.inv_direction.x;
    tymin = (aabb[ray.sign[1]].y - ray.origin.y) * ray.inv_direction.y;
    tymax = (aabb[1-ray.sign[1]].y - ray.origin.y) * ray.inv_direction.y;
    tzmin = (aabb[ray.sign[2]].z - ray.origin.z) * ray.inv_direction.z;
    tzmax = (aabb[1-ray.sign[2]].z - ray.origin.z) * ray.inv_direction.z;
    tmin = max(max(tmin, tymin), tzmin);
    tmax = min(min(tmax, tymax), tzmax);
	return (tmin <= tmax); // we have an intersection; no intersection if tmin > tmax
}


const float EPSILON  = 1e-6;
// std. moeller trumbore triangle intersection test
bool intersectTriangle(const in Ray r, const in mat3 triangle, const in float tfar, out float t, out vec2 uv) {
	vec3 e0 = triangle[1] - triangle[0];
	vec3 e1 = triangle[2] - triangle[0];
	vec3 pvec = cross(r.direction, e1);
	float det = dot(e0, pvec);
	if(abs(det) < EPSILON) // intersect backfaces
	//if(a < EPSILON) // skip backfaces
		return false;
	float f = 1.0 / det;
	vec3  s = r.origin - triangle[0];
	float u = f * dot(s, pvec);

	if(u < 0.0 || u > 1.0)
		return false;

	vec3  qvec = cross(s, e0);
	float v = f * dot(r.direction, qvec);
	if(v < 0.0 || u + v > 1.0)
		return false;
	t = f * dot(e1, qvec);

	if (t < EPSILON)
		return false;

	uv = vec2(u, v);
	return (t > 0.0) && (t < tfar);
}

vec3 computeTriangleNormal(const in mat3 triangle) {
	vec3 e0 = triangle[1] - triangle[0];
	vec3 e1 = triangle[2] - triangle[0];
	return normalize(cross(e1, e0));
}

struct HitInfo {
	int triIndex;
	float tfar;
	vec2 uv;
};

struct MaterialData {
	vec3 diffuseColor;
	float roughness;
};

struct RenderState {
	vec3 hitPoint;
	vec3 geometryNormal;
	mat3 shadingONB;
	vec3 outDir; // direction towards camera
	vec3 inDir;  // direction towrads light/bounce
	MaterialData material;
};


void getMaterialData(const in int triIndex, out MaterialData mat) {
	int matIndex = int(texelFetch(u_sampler2D_TriangleData, ivec2(0, triIndex), 0).w);
	vec4 matData = texelFetch(u_sampler2D_MaterialData, ivec2(0, matIndex), 0).xyzw;
	mat.diffuseColor = matData.xyz;
	mat.roughness = matData.w;
}

mat3 getSceneTriangle(const in int index) {
	mat3 triangle;
	triangle[0] = texelFetch(u_sampler2D_TriangleData, ivec2(0, index), 0).xyz;
	triangle[1] = texelFetch(u_sampler2D_TriangleData, ivec2(1, index), 0).xyz;
	triangle[2] = texelFetch(u_sampler2D_TriangleData, ivec2(2, index), 0).xyz;
	return triangle;
}


bool bvh_IntersectRayBox(const in Ray r, const in float tfar, int pn, out int si, out int ei) {
	vec4 nodeA = texelFetch(u_sampler2D_BVHData, ivec2(0, pn), 0);
	vec4 nodeB = texelFetch(u_sampler2D_BVHData, ivec2(1, pn), 0);
	vec3 aabb[2];
	aabb[0] = nodeA.xyz;
	aabb[1] = nodeB.xyz;
	si = int(nodeA.w);
	ei = int(nodeB.w);

	float tmin, tmax;
	bool hasHit = intersectAABB(r, aabb, tmin, tmax);
	return hasHit && ((tmin <= tfar) || (tmin < 0.0 && tmax <= tfar));


	//!!TODO: check if this has correct semantics for rays that start inside the box?
	//return hasHit && ((/*tmin > 0.0 && */tmin <= tfar) || (tmin < 0.0 && tmax <= tfar));
}


/*!!TODO/!!TOOPT:
    - sort out the many tfars
    - sort the needed data in ray payload and return values
*/
bool intersectSceneTriangles_BVH(const in Ray r, out HitInfo hit) {
	hit.tfar = r.tfar;
	hit.triIndex = -1;


	int stack[16];
	int top = 1;
	int pn = 0;

	float tfar = r.tfar;
	bool foundHit = false;
	int si, ei;

	while (top > 0) {
		if (bvh_IntersectRayBox(r, tfar, pn, si, ei)) {
			
			if (si > 0) { // intermediate node
				// !!TOOPT: sort front to back based on ray sign (but this needs additional data in nodes based on construction)
				pn = si;
				stack[top++] = ei;
			} else { // leaf node
				for (int i = -si; i < -ei; i++) {
					float t = 0.0;
					vec2 uv;
					mat3 triangle = getSceneTriangle(i);
					if (intersectTriangle(r, triangle, hit.tfar, t, uv)) {
						hit.tfar = t;
						hit.triIndex = i;
						hit.uv = uv;
						foundHit = true;
						tfar = t;
					}
				}

				pn = stack[--top];
			}

		} else {
			pn = stack[--top];
		}
	}

	return foundHit;
}

bool intersectSceneTriangles_Bruteforce(const in Ray r, out HitInfo hit) {
	hit.tfar = r.tfar;
	hit.triIndex = -1;

	for (int i = 0; i < u_int_NumTriangles; i++) {
		mat3 triangle = getSceneTriangle(i);

		float t = 0.0;
		vec2 uv;
		if (intersectTriangle(r, triangle, hit.tfar, t, uv)) {
			hit.tfar = t;
			hit.triIndex = i;
			hit.uv = uv;
		} else {
		}
	}

	return hit.triIndex >= 0;
}

bool intersectScene_Nearest(const in Ray r, out HitInfo hit) {
	//return intersectSceneTriangles_Bruteforce(r, hit);
	return intersectSceneTriangles_BVH(r, hit);
}

bool isVisible(const in vec3 p0, const in vec3 p1) {
	//Ray r = createRay(p1-p0, p0, 1.0);
	Ray r = createRay(normalize(p1-p0), p0, length(p1-p0));

	HitInfo hit;
	return !intersectScene_Nearest(r, hit); //!!TOOPT: add an early hit function here

	/*
	for (int i = 0; i < u_int_NumTriangles; i++) {
		mat3 triangle = getSceneTriangle(i);
		float t = 0.0;
		vec2 uv;
		if (intersectTriangle(r, triangle, r.tfar, t, uv)) {
			return false;
		}
	}

	return true;
	*/
}




void fillRenderState(const in Ray r, const in HitInfo hit, out RenderState rs) {
	rs.geometryNormal = computeTriangleNormal(getSceneTriangle(hit.triIndex));
	rs.geometryNormal *= -sign(dot(rs.geometryNormal, r.direction));

	getMaterialData(hit.triIndex, rs.material);
	

	rs.shadingONB = computeONB(rs.geometryNormal);

	rs.hitPoint = r.origin + r.direction * hit.tfar + rs.geometryNormal*0.0001;
	rs.outDir = -r.direction;
}

const float TFAR_MAX = 10000.0;


vec3 brdf_lambert_evaluate(out float o_pdf, const in RenderState rs, const in vec3 wi) {
	o_pdf = max(dot(wi, rs.geometryNormal), 0.0) * INV_PI;
	return rs.material.diffuseColor * INV_PI;
}

bool brdf_lambert_sample(out vec3 wi, const in RenderState rs) {
	wi = rs.shadingONB * hemisphereSample_cos(rng_NextFloat(), rng_NextFloat());
	return true;
}

vec3 brdf_mirror_evaluate(out float o_pdf, const in RenderState rs, const in vec3 wi) {
	o_pdf = 0.0;
	return rs.material.diffuseColor;
}

bool brdf_mirror_sample(out vec3 wi, const in RenderState rs) {
	wi = -reflect(rs.outDir, rs.geometryNormal);
	return true;
}


vec3 brdf_phong_evaluate(out float o_pdf, const in RenderState rs, in vec3 wi) {
    float cos_NI = dot(rs.geometryNormal, wi);
    float cos_NO = dot(rs.geometryNormal, rs.outDir);

	float exponent = rs.material.roughness * 510.0;

    if (cos_NI > 0.0 && cos_NO > 0.0) {
        vec3 R = -reflect(rs.outDir, rs.geometryNormal);
        float cosRI = dot(R, wi);
        if (cosRI > 0.0) {
            o_pdf = (exponent + 1.0) * float(INV_PI / 2.0) * pow(cosRI, exponent);
            return rs.material.diffuseColor * cos_NI * (exponent + 2.0) / (exponent + 1.0);
        }
    }
    o_pdf = 0.0;
	return vec3(0.0);
}

bool brdf_phong_sample(out vec3 wi, const in RenderState rs) {
	float cos_NO = dot(rs.geometryNormal, rs.outDir);
	float exponent = rs.material.roughness * 510.0;
	if (cos_NO > 0.0) {
		float rx = rng_NextFloat();
		float ry = rng_NextFloat();
		mat3 onb = computeONB(-reflect(rs.outDir, rs.geometryNormal));
		float phi = 2.0 * PI * rx;
		float sp = sin(phi);
		float cp = cos(phi);
		float cosTheta = pow(ry, 1.0 / (exponent + 1.0));
		float sinTheta2 = 1.0 - cosTheta * cosTheta;
		float sinTheta = (sinTheta2 > 0.0) ? sqrt(sinTheta2) : 0.0;
		wi = onb * vec3(cp * sinTheta, sp * sinTheta, cosTheta);
		float cos_NI = dot(rs.geometryNormal, wi);
		if (cos_NI > 0.0) {
			return true;
			//return cos_NI * (exponent + 2.0) / (exponent + 1.0);
		}
	}
	return false;
}



int sampleBSDFBounce(inout RenderState rs, inout vec3 pathWeight) {
	float pdf = 0.0;
	if (rs.material.roughness == 0.0) {
		if (brdf_lambert_sample(rs.inDir, rs)) {
			pathWeight *= brdf_lambert_evaluate(pdf, rs, rs.inDir);
		} else return -1;
	} else if (rs.material.roughness < 1.0) {
		if (brdf_phong_sample(rs.inDir, rs)) {
			pathWeight *= brdf_phong_evaluate(pdf, rs, rs.inDir);
		} else return -1;
	} else {
		if (brdf_mirror_sample(rs.inDir, rs)) {
			pathWeight *= brdf_mirror_evaluate(pdf, rs, rs.inDir);
		} else return -1;
	}

	Ray r = createRay(rs.inDir, rs.hitPoint, TFAR_MAX);
	HitInfo hit;

	if (intersectScene_Nearest(r, hit)) {
		fillRenderState(r, hit, rs);
		return 1;
	}

	return 0;
}

uniform vec3 u_vec3_PointLightColor;
uniform vec3 u_vec3_PointLightPosition;
uniform float u_float_PointLightIntensity;

uniform int u_int_MaxBounceDepth;

vec3 sampleAndEvaluateDirectLight(const in RenderState rs) {
	vec3 pointLightColor = u_vec3_PointLightColor * u_float_PointLightIntensity;
	float pdf = 0.0;
	if (rs.material.roughness < 0.9) {
		vec3 d = u_vec3_PointLightPosition - rs.hitPoint;
		float cosNL = dot(d, rs.geometryNormal);

		if (cosNL > 0.0) {
			if (isVisible(rs.hitPoint, u_vec3_PointLightPosition)) {
				float distance2 = dot(d, d);
				d = normalize(d);

				if (rs.material.roughness == 0.0) {
					return brdf_lambert_evaluate(pdf, rs, d) * (pointLightColor / distance2);
				} /*else {
					return brdf_phong_evaluate(pdf, rs, d) * (pointLightColor / distance2);
				}*/
			}
		}
	} else { // singular
		
	}


	return vec3(0.0);
}

void main() {
	init_RNG();

	// box filter
	vec2 pixelOffset = (vec2(rng_NextFloat(), rng_NextFloat()) * 2.0) * u_vec2_InverseResolution;

	vec3 pixelDirection = normalize(vec3(v_PixelCoord + pixelOffset, -1.0));
	
	pixelDirection = pixelDirection * mat3(u_mat4_ViewMatrix); // transposed mult == inverse
	vec3 origin = -u_mat4_ViewMatrix[3].xyz * mat3(u_mat4_ViewMatrix); // !!TOOPT: maybe faster to use uniform instead of compute origin here?

	Ray r = createRay(pixelDirection, origin, TFAR_MAX);
	HitInfo hit;

	vec3 pathWeight = vec3(1.0);
	vec3 colorAccum = vec3(0.0);


	if (intersectScene_Nearest(r, hit)) { // primary camera ray
		RenderState rs;
		fillRenderState(r, hit, rs);

		colorAccum += sampleAndEvaluateDirectLight(rs) * pathWeight;

		for (int depth = 0; depth < u_int_MaxBounceDepth; depth++) {
			int bounceType = sampleBSDFBounce(rs, pathWeight);

			if (bounceType == -1) { // absorbed
			}
			if (bounceType == 0) { // background
				//colorAccum += vec3(1.0) * pathWeight;
				break;
			}
			colorAccum += sampleAndEvaluateDirectLight(rs) * pathWeight;


		}

		//outColor = vec4(rs.geometryNormal * 0.5 + vec3(0.5), 1.0);
		//outColor = vec4(rs.geometryNormal * 0.5 + vec3(0.5), 1.0);
		//outColor = vec4(mod(abs(rs.hitPoint), 1.0), 1.0);
		//outColor = vec4(float((hit.triIndex+3)%4)/4.0, float((hit.triIndex*3)%7)/7.0, float(hit.triIndex%13)/13.0, 1.0);
		//outColor = vec4(float((hit.triIndex+3)%4)/4.0 * hit.uv.x, float((hit.triIndex*3)%7)/7.0 * (1.0 -  hit.uv.y - hit.uv.x), float(hit.triIndex%13)/13.0 * hit.uv.y, 1.0);
	} else { // direct background hit
		
	}


	outColor = vec4(colorAccum, 1.0);

	//outColor = vec4(1.0, 0.0, 0.0, 1.0);
	//outColor = vec4(rayDir, 1.0);
	//outColor = texture(u_sampler2D_TriangleData, vec2(0.0, 0.0));
	//outColor = texelFetch(u_sampler2D_TriangleData, ivec2(0, 0), 0);
}
`;


function logI(txt) {
	console.log(txt);
}

function logE(e, txt) {
	console.log("ERROR: ("+e+"): "+txt);
}


function createShader(gl, type, source) {
	let shader = gl.createShader(type);
	gl.shaderSource(shader, source);
	gl.compileShader(shader);
	if (gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
		return shader;
	} else {
		logE("shader compile error", gl.getShaderInfoLog(shader));
		gl.deleteShader(shader);
		return undefined;
	}
}

function createProgramFromSource(gl, vertexShaderSource, fragmentShaderSource) {
	let vs = createShader(gl, gl.VERTEX_SHADER, vertexShaderSource);
	let fs = createShader(gl, gl.FRAGMENT_SHADER, fragmentShaderSource);
	if (vs !== undefined && fs !== undefined) {
		let program = gl.createProgram();
		gl.attachShader(program, vs);
		gl.attachShader(program, fs);
		gl.linkProgram(program);
		if (gl.getProgramParameter(program, gl.LINK_STATUS)) {
			return program;
		} else {
			logE("program link error", gl.getProgramInfoLog(program));
			gl.deleteProgram(program);
			return undefined;
		}
	} else {
		return undefined;
	}
}

function printGLInfo(gl) {
	logI("gl.VENDOR = " + gl.getParameter(gl.VENDOR));
	logI("gl.RENDERER = " + gl.getParameter(gl.RENDERER));
	logI("gl.VERSION = " + gl.getParameter(gl.VERSION));
	logI("gl.MAX_TEXTURE_SIZE = " + gl.getParameter(gl.MAX_TEXTURE_SIZE));
	logI("gl.MAX_3D_TEXTURE_SIZE = " + gl.getParameter(gl.MAX_3D_TEXTURE_SIZE));
	console.log(gl.getSupportedExtensions());
}



function helper_PrintTriangleData(triData, vtxStride) {
	let numTris = triData.length / (3 * vtxStride) | 0;
	for (let i = 0; i < numTris; i++) {
		let i0 = i * (3 * vtxStride);
		let i1 = i * (3 * vtxStride) + vtxStride;
		let i2 = i * (3 * vtxStride) + vtxStride + vtxStride;
		console.log("(" + triData[i0+0] + ","  + triData[i0+1] + ","  + triData[i0+2] + ") " +
					"(" + triData[i1+0] + ","  + triData[i1+1] + ","  + triData[i1+2] + ") " +
					"(" + triData[i2+0] + ","  + triData[i2+1] + ","  + triData[i2+2] + ") ");
	}
}


function createDataTextureRGBA32F(gl, data, sX) {
	let sY = data.length / (sX * 4) | 0;
	let tex = gl.createTexture();
	gl.activeTexture(gl.TEXTURE0);
	gl.bindTexture(gl.TEXTURE_2D, tex);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
	gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, sX, sY, 0, gl.RGBA, gl.FLOAT, data);
	return tex;
}

function createDataTextureRGB32F(gl, data, sX) {
	let sY = data.length / (sX * 3) | 0;
	let tex = gl.createTexture();
	gl.activeTexture(gl.TEXTURE0);
	gl.bindTexture(gl.TEXTURE_2D, tex);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
	gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
	gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB32F, sX, sY, 0, gl.RGB, gl.FLOAT, data);
	return tex;
}

function loadSceneIntoTexture(scene) {
	let gl = scene.gl;

	let _triangleData = SCENE_TriangleData;
	let _materialData = SCENE_MaterialData;
	

	const vtxDataStride = 4;

	scene.numTriangles = _triangleData.length / (3*vtxDataStride);

	scene.triBVH = new SimpleTriangleBVH(vtxDataStride);
	scene.triBVH.build(_triangleData);

	// now we need to reorder the tri data based on the bvh indices created during construction
	//!!TOOPT: do thi sin place
	let origData = _triangleData;

	console.log("NumTriangles = " + scene.numTriangles);
	//console.log(scene.triBVH.m_pTriIndices);

	// reorder triangles based on bvh index array
	let _triData = new Float32Array(origData.length);
	for (let i = 0; i < scene.numTriangles; i++) {
		let srcIdx = scene.triBVH.m_pTriIndices[i];
		for (let j = 0; j < (3*vtxDataStride); j++) {
			_triData[i*(3*vtxDataStride)+j] = origData[srcIdx*(3*vtxDataStride)+j];
		}
	}

	let flatBVHdata = scene.triBVH.createAndCopyToFlattenedArray_StandardFormat();
	scene.bvhTexture = createDataTextureRGBA32F(gl, flatBVHdata, 2);

	scene.triangleTexture = createDataTextureRGBA32F(gl, _triData, 3);
	scene.materialTexture = createDataTextureRGBA32F(gl, _materialData, 1);



}


function GLSLRayTracer(gl, scene, canvas) {
	let me = this;

	me.gl = gl;

	me.resizeAccumFramebuffer = function() {
		if (!me.accumTexture || scene.camera.width != canvas.width || scene.camera.height != canvas.height) {
			canvas.width = scene.camera.width;
			canvas.height = scene.camera.height;

			if (me.accumTexture) gl.deleteTexture(me.accumTexture);

			me.width = canvas.width;
			me.height = canvas.height;

			console.log("Resizing AccumFramebuffer to " + me.width + "x" + me.height);

			me.accumTexture = gl.createTexture();
			gl.bindTexture(gl.TEXTURE_2D, me.accumTexture);
			gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
			gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
			gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
			gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
			gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, canvas.width, canvas.height, 0, gl.RGBA, gl.FLOAT, null);
			//gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, canvas.width, canvas.height, 0, gl.RGBA, gl.FLOAT, null);
			//gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, canvas.width, canvas.height, 0, gl.RGB, gl.UNSIGNED_BYTE, null);
			gl.bindTexture(gl.TEXTURE_2D, null);
			gl.bindFramebuffer(gl.FRAMEBUFFER, me.accumFBO);
			gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, me.accumTexture, 0);
			//console.log(gl.checkFramebufferStatus(gl.FRAMEBUFFER));
			gl.bindFramebuffer(gl.FRAMEBUFFER, null);
		}
	}

	//-------
	// setup fbo for accumulation
	me.accumFBO = gl.createFramebuffer();
	//-------


	me.program_Display = createProgramFromSource(gl, shaderCode_Display_vs, shaderCode_Display_fs);
	let loc_displayTex = gl.getUniformLocation(me.program_Display, "u_sampler2D_ImageToDisplay");
	me.loc_display_u_float_InverseFrameNumber = gl.getUniformLocation(me.program_Display, "u_float_InverseFrameNumber");
	gl.useProgram(me.program_Display);
	gl.uniform1i(loc_displayTex, 0);
	gl.useProgram(null);

	//-------
	me.program_PathTracer = createProgramFromSource(gl, shaderCode_PathTracer_vs, shaderCode_PathTracer_fs);

	this.positionLocation = gl.getAttribLocation(me.program_PathTracer, "a_position");

	this.triangleDataLocation = gl.getUniformLocation(me.program_PathTracer, "u_sampler2D_TriangleData");
	this.materialDataLocation = gl.getUniformLocation(me.program_PathTracer, "u_sampler2D_MaterialData");
	this.bvhDataLocation = gl.getUniformLocation(me.program_PathTracer, "u_sampler2D_BVHData");
	this.loc_u_vec2_InverseResolution = gl.getUniformLocation(me.program_PathTracer, "u_vec2_InverseResolution");
	this.numTriangleLocation = gl.getUniformLocation(me.program_PathTracer, "u_int_NumTriangles");
	this.frameNumberLocation = gl.getUniformLocation(me.program_PathTracer, "u_int_FrameNumber");
	this.viewMatrixLocation = gl.getUniformLocation(me.program_PathTracer, "u_mat4_ViewMatrix");

	this.loc_u_vec3_PointLightColor = gl.getUniformLocation(me.program_PathTracer, "u_vec3_PointLightColor");
	this.loc_u_vec3_PointLightPosition = gl.getUniformLocation(me.program_PathTracer, "u_vec3_PointLightPosition");
	this.loc_u_float_PointLightIntensity = gl.getUniformLocation(me.program_PathTracer, "u_float_PointLightIntensity");
	this.loc_u_int_MaxBounceDepth = gl.getUniformLocation(me.program_PathTracer, "u_int_MaxBounceDepth");
	//-------

	this.positionBuffer = gl.createBuffer();
	gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
	const positions = new Float32Array([-1.0, -1.0,  -1.0, 1.0,  1.0, -1.0,  1.0, 1.0]);
	gl.bufferData(gl.ARRAY_BUFFER, positions, gl.STATIC_DRAW);

	this.vao = gl.createVertexArray();
	gl.bindVertexArray(this.vao);
	gl.enableVertexAttribArray(this.positionLocation);
	gl.vertexAttribPointer(this.positionLocation, 2, gl.FLOAT, false, 0, 0);

	gl.useProgram(me.program_PathTracer);
	gl.uniform1i(this.triangleDataLocation, 0);
	gl.uniform1i(this.materialDataLocation, 1);
	gl.uniform1i(this.bvhDataLocation, 2);
	gl.uniform1i(me.numTriangleLocation, scene.numTriangles);

	gl.useProgram(null);



	me.renderFrame = function (camera, canvas, frameNumber, g_RenderSettings) {
		let gl = me.gl;

		me.resizeAccumFramebuffer();


		gl.activeTexture(gl.TEXTURE0);
		gl.bindTexture(gl.TEXTURE_2D, scene.triangleTexture);
		gl.activeTexture(gl.TEXTURE1);
		gl.bindTexture(gl.TEXTURE_2D, scene.materialTexture);
		gl.activeTexture(gl.TEXTURE2);
		gl.bindTexture(gl.TEXTURE_2D, scene.bvhTexture);

		gl.useProgram(me.program_PathTracer);
		gl.uniformMatrix4fv(me.viewMatrixLocation, false, camera.viewMatrix);
		gl.uniform1i(me.frameNumberLocation, frameNumber);

		gl.uniform2f(me.loc_u_vec2_InverseResolution, 1.0 / me.width, 1.0 / me.height);
		gl.uniform3f(me.loc_u_vec3_PointLightPosition, scene.pointLightPosition[0], scene.pointLightPosition[1], scene.pointLightPosition[2]);
		gl.uniform3f(me.loc_u_vec3_PointLightColor, scene.pointLightColor[0]/255.0, scene.pointLightColor[1]/255.0, scene.pointLightColor[2]/255.0);
		gl.uniform1f(me.loc_u_float_PointLightIntensity, scene.pointLightIntensity);
		gl.uniform1i(me.loc_u_int_MaxBounceDepth, g_RenderSettings.bounceDepth);

		gl.bindFramebuffer(gl.FRAMEBUFFER, me.accumFBO);
		gl.viewport(0, 0, canvas.width, canvas.height);

		if (frameNumber == 0) {
			gl.clear(gl.COLOR_BUFFER_BIT);
		}

		gl.blendFunc(gl.ONE, gl.ONE);
		gl.enable(gl.BLEND);


		gl.bindVertexArray(me.vao);
		gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);

		gl.disable(gl.BLEND);
		//gl.clear(GL_COLOR_BUFFER_BIT);

		gl.bindFramebuffer(gl.FRAMEBUFFER, null);
		gl.viewport(0, 0, canvas.width, canvas.height);

		gl.activeTexture(gl.TEXTURE0);
		gl.bindTexture(gl.TEXTURE_2D, me.accumTexture);

		gl.useProgram(me.program_Display);
		let invFrameNumber = 1.0 / (frameNumber+1.0);
		//console.log(invFrameNumber);
		gl.uniform1f(me.loc_display_u_float_InverseFrameNumber, invFrameNumber);
		gl.bindVertexArray(me.vao);
		gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);


	}
}

function fromThetaPhi(out, theta,  phi) {
	vec3.set(out, Math.sin(theta) * Math.cos(phi), Math.sin(theta) * Math.sin(phi), Math.cos(theta));
}


function Scene(gl) {
	let me = this;
	me.gl = gl;

	me.lastFrameChanged = true;

	loadSceneIntoTexture(me);

	me.camera = {};

	me.camera.rotX = 0; //-1.8;
	me.camera.rotY = -0.2;
	me.camera.dist = 8.0;
	me.camera.eye = vec3.create();
	me.camera.at = vec3.create();
	me.camera.up = vec3.create();
	me.camera.viewMatrix = mat4.create();

	me.camera.width = 512;
	me.camera.height = 512;

	vec3.set(me.camera.at, 0.0, 0.0, 0.0);
	vec3.set(me.camera.up, 0.0, 0.0, 1.0);

	me.process = function(dt) {
		me.lastFrameChanged = false;
	}

	me.guiChangeEvent = function() {
		me.camera.width |= 0;
		me.camera.height |= 0;

		me.lastFrameChanged = true;
	}

	me.pointLightPosition = [0, -1, 3];
	me.pointLightColor = [255, 255, 255];
	me.pointLightIntensity = 50.0;

	me.fillGUI = function(gui) {
		gui.add(me.camera, "width", 1, 1024).name("resX").onChange(me.guiChangeEvent);
		gui.add(me.camera, "height", 1, 1024).name("resY").onChange(me.guiChangeEvent);
		gui.add(me.pointLightPosition, "0", -5, 5).name("LightPosX").onChange(me.guiChangeEvent);
		gui.add(me.pointLightPosition, "1", -5, 5).name("LightPosY").onChange(me.guiChangeEvent);
		gui.add(me.pointLightPosition, "2", -1, 10).name("LightPosZ").onChange(me.guiChangeEvent);
		gui.add(me, "pointLightIntensity", 0.0, 200.0).onChange(me.guiChangeEvent);
		gui.addColor(me, "pointLightColor").onChange(me.guiChangeEvent);
	}

	me.rotate = function(mX, mY) {
		if (mX == 0 && mY == 0) return;
		me.camera.rotX -= mX / 128.0;
		me.camera.rotY -= mY / 128.0;
		me.updateViewMatrix();
		me.lastFrameChanged = true;
	}

	me.zoom = function(mY) {
		if (mY == 0) return;
		me.camera.dist += mY * Math.max(me.camera.dist, 0.01) * 0.01;
		me.updateViewMatrix();
		me.lastFrameChanged = true;
	}

	me.updateViewMatrix = function() {
		const yRotLimit = 0.48*Math.PI
		if (me.camera.rotX < 0.0) me.camera.rotX += 2.0*Math.PI;
		if (me.camera.rotY < -yRotLimit) me.camera.rotY = -yRotLimit;
		if (me.camera.rotY > yRotLimit) me.camera.rotY = yRotLimit;
		if (me.camera.dist < 0.0) me.camera.dist = 0.0;

		fromThetaPhi(me.camera.eye, me.camera.rotY + 0.5*Math.PI, me.camera.rotX);
		vec3.scale(me.camera.eye, me.camera.eye, me.camera.dist);
		vec3.add(me.camera.eye, me.camera.eye, me.camera.at);
		mat4.lookAt(me.camera.viewMatrix, me.camera.eye, me.camera.at, me.camera.up);
	}

	me.updateViewMatrix();

}

let scene, glslRayTracer;


function PathTracingRenderer(glslRayTracer, scene, infoDiv) {
	let me = this;

	me.glslRayTracer = glslRayTracer;
	me.scene = scene;

	me.m_Param_ProgressiveEnabled = false;
	me.bounceDepth = 1;

	me.frameNumber = 0;

	me.renderFrame = function() {

		if (me.scene.lastFrameChanged) {
			me.frameNumber = 0;
		}

		glslRayTracer.renderFrame(scene.camera, canvas, me.frameNumber, me);

		me.frameNumber++;
		me.scene.process(0.0); // for now no real scene processing
	}


	me.startTime = Date.now();
	me.fps = 0.0;

	me.updateInfoDiv = function() {
		infoDiv.innerHTML = "Info <br />";
		infoDiv.innerHTML += "Frame Number = " + me.frameNumber + "<br />";
		infoDiv.innerHTML += "Avg. FPS = " + me.fps.toFixed(2) + "<br />";
	}

	me.progressiveRender = function() {

		if (me.m_Param_ProgressiveEnabled || me.scene.lastFrameChanged) {
			if (me.m_Param_ProgressiveEnabled) {
				if (me.startTime == undefined || me.scene.lastFrameChanged) {
					me.startTime = Date.now();
					me.fpsFrameCount = 0;
				}
			} else {
				me.startTime = undefined;
			}

			me.renderFrame();

			if (me.startTime) {
				me.deltaTime = (Date.now() - me.startTime) / 1000.0;
				if (me.deltaTime > 0.0) {
					me.fpsFrameCount++;
					me.fps = me.fpsFrameCount / me.deltaTime;
					if (me.fpsFrameCount > 0) {
						me.fpsFrameCount = 0;
						me.startTime = Date.now();
					}
				} else {
					me.fps = 0.0;
				}
			} else {
				me.fps = 0.0;
			}

			me.updateInfoDiv();
		}

		window.requestAnimationFrame(me.progressiveRender);
	}


	me.guiChangeEvent = function() {
		me.scene.lastFrameChanged = true;

		/*if (me.m_Param_ProgressiveEnabled) {
			me.progressiveRender();
		} else {
			me.renderFrame();
		}*/
	}

	me.fillGUI = function(gui) {
		gui.add(me, "bounceDepth", 0, 4).step(1).onChange(me.guiChangeEvent);
		gui.add(me, "m_Param_ProgressiveEnabled").name("Progressive");//.onChange(me.guiChangeEvent);
	}


	me.processMouseEvents = function(e) {
		if (e.buttons != 0) {
			if (e.button == 0) {
				me.scene.rotate(e.movementX, e.movementY);
			} else if (e.button == 1) {
				me.scene.zoom(e.movementY);
			}
		}
	}
}


function initGUI(pt, scene) {
	let ptgui = new dat.GUI({ autoPlace: false });
	pt.fillGUI(ptgui);
	document.getElementById("renderSettings").appendChild(ptgui.domElement);

	let scenegui = new dat.GUI({ autoPlace: false });
	scene.fillGUI(scenegui);
	document.getElementById("sceneSettings").appendChild(scenegui.domElement);
}


function handleObjFileSelect(evt) {
	let reader = new FileReader();
	reader.onerror = function(e) {
		logE(e, "OBJ File Error");
	}
	
	reader.onload = function(e) {
		console.log(e.target.result);
	}

	reader.readAsText(evt.target.files[0]);
}

function main() {
	canvas = document.getElementById("webglcanvas");
	if (!(gl = canvas.getContext("webgl2"))) {
		logE("GL", "No WebGL2");
		return;
	}

	printGLInfo(gl);

	if (gl.getExtension("EXT_color_buffer_float") == null) {
		document.body.innerHTML = "ERROR: EXT_color_buffer_float not supported";
		return;
	}

	//document.getElementById('objFileInput').addEventListener('change', handleObjFileSelect, false);


	logI("canvas.width = " + canvas.width + ", canvas.height = " + canvas.height);

	//gl.clearColor(0.0, 0.0, 0.0, 1.0);
	//gl.clear(gl.COLOR_BUFFER_BIT);
	
	scene = new Scene(gl);
	glslRayTracer = new GLSLRayTracer(gl, scene, canvas);

	let pathTracingRenderer = new PathTracingRenderer(glslRayTracer, scene, document.getElementById("infoDiv"));

	initGUI(pathTracingRenderer, scene);
	canvas.addEventListener("mousemove", pathTracingRenderer.processMouseEvents);

	pathTracingRenderer.progressiveRender();


	/*for (let i = 0; i < 1; i++) {
		glslRayTracer.renderFrame(scene.camera, canvas, i);
	}*/

	//scene.renderFrame = function() {
	//	glslRayTracer.renderFrame(scene.camera, canvas, 0/*frameNumber*/);
	//}*


}
