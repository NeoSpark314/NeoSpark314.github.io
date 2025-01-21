// my simple MathUtils module
let MathUtils = (function() {
	let AABB = {};
	let Triangle = {};
	let Vec3f = {};
	

	AABB.create = function() {
		let ret = new Float32Array(8); // to be SIMD friendly for the future we use 4 floats per entry;
		ret[0] = 0.0; ret[1] = 0.0; ret[2] = 0.0; ret[3] = 0.0;
		ret[4] = 0.0; ret[5] = 0.0; ret[6] = 0.0; ret[7] = 0.0;
		return ret;
	}

	AABB.reset = function(o_AABB) {
		AABB.setMin1f(o_AABB, MathUtils.FLT_MAX);
		AABB.setMax1f(o_AABB, -MathUtils.FLT_MAX);
	}

	AABB.setMin1f = function(o_AABB, m) {
		o_AABB[0] = m;
		o_AABB[1] = m;
		o_AABB[2] = m;
	}

	AABB.setMax1f = function(o_AABB, m) {
		o_AABB[4] = m;
		o_AABB[5] = m;
		o_AABB[6] = m;
	}

	AABB.setMin3f = function(o_AABB, mx, my, mz) {
		o_AABB[0] = mx;
		o_AABB[1] = my;
		o_AABB[2] = mz;
	}

	AABB.setMax3f = function(o_AABB, mx, my, mz) {
		o_AABB[4] = mx;
		o_AABB[5] = my;
		o_AABB[6] = mz;
	}

	AABB.computeCenterVec3f = function(i_AABB, o_Vec3f) {
		o_Vec3f[0] = (i_AABB[4] + i_AABB[0])*0.5;
		o_Vec3f[1] = (i_AABB[5] + i_AABB[1])*0.5;
		o_Vec3f[2] = (i_AABB[6] + i_AABB[2])*0.5;
	}

	AABB.computeExtendVec3f = function(i_AABB, o_Vec3f) {
		o_Vec3f[0] = i_AABB[4] - i_AABB[0];
		o_Vec3f[1] = i_AABB[5] - i_AABB[1];
		o_Vec3f[2] = i_AABB[6] - i_AABB[2];
	}

	AABB.getMaxExtendIdx = function(i_AABB) {
		let dx = i_AABB[4] - i_AABB[0];
		let dy = i_AABB[5] - i_AABB[1];
		let dz = i_AABB[6] - i_AABB[2];
		if (dx > dy) {
			if (dx > dz) return 0;
			else return 2;
		} else if (dy > dz) return 1;
		else return 2;
	}

	AABB.expandTriangle = function (o_AABB, i_Triangle) {
		AABB.expand3f(o_AABB, i_Triangle[0], i_Triangle[1], i_Triangle[2]);
		AABB.expand3f(o_AABB, i_Triangle[3], i_Triangle[4], i_Triangle[5]);
		AABB.expand3f(o_AABB, i_Triangle[6], i_Triangle[7], i_Triangle[8]);
	}

	AABB.expand3f = function (o_AABB, vx, vy, vz) {
		o_AABB[0] = Math.min(o_AABB[0], vx);
		o_AABB[1] = Math.min(o_AABB[1], vy);
		o_AABB[2] = Math.min(o_AABB[2], vz);
		o_AABB[4] = Math.max(o_AABB[4], vx);
		o_AABB[5] = Math.max(o_AABB[5], vy);
		o_AABB[6] = Math.max(o_AABB[6], vz);
	}

	AABB.expandVec3f = function(o_AABB, i_Vec3f) {
		o_AABB[0] = Math.min(o_AABB[0], i_Vec3f[0]);
		o_AABB[1] = Math.min(o_AABB[1], i_Vec3f[1]);
		o_AABB[2] = Math.min(o_AABB[2], i_Vec3f[2]);
		o_AABB[4] = Math.max(o_AABB[4], i_Vec3f[0]);
		o_AABB[5] = Math.max(o_AABB[5], i_Vec3f[1]);
		o_AABB[6] = Math.max(o_AABB[6], i_Vec3f[2]);
	}


	Triangle.create = function() {
		let ret = new Float32Array(9);
		for (let i = 0; i < 9; i++) ret[i] = 0.0;
		return ret;
	}

	Triangle.computeCenterVec3f = function(i_Triangle, o_Vec3f) {
		o_Vec3f[0] = (i_Triangle[0] + i_Triangle[3] + i_Triangle[6]) * (1.0/3.0);
		o_Vec3f[1] = (i_Triangle[1] + i_Triangle[4] + i_Triangle[7]) * (1.0/3.0);
		o_Vec3f[2] = (i_Triangle[2] + i_Triangle[5] + i_Triangle[8]) * (1.0/3.0);
	}

	Vec3f.create = function() {
		let ret = new Float32Array(3);
		ret[0] = 0.0; ret[1] = 0.0; ret[2] = 0.0;
		return ret;
	}
	
	return Object.freeze({
		FLT_MAX: 3.40282347e+38,

		AABB : Object.freeze(AABB),
		Triangle: Object.freeze(Triangle),
		Vec3f: Object.freeze(Vec3f),
		

	});
	

})();
