// Simple BVH support classes
// Author: Holger Dammertz (holger.dammertz.org)
//

function SimpleTriangleBVH(triDataStride) {
	let me = this;

	me.getShaderTraversalCodeString = function() {
		return `

		`;
	}

	let BVHNode = function(nodeIdx) {
		this.m_NodeIdx = nodeIdx; // this is to simplify flattening; it is not very memory efficient
		this.m_AABB = MathUtils.AABB.create();
		this.m_Left = null;
		this.m_Right = null;
		this.m_Leaf_si = -1;
		this.m_Leaf_ei = -1;
	}

	me.m_Param_MaxTreeDepth = -1; // -1 disables depth limit
	me.m_Param_MaxNumLeafTris = 8; 

	me.m_pTriangles = null;
	me.m_NumTriangles = 0;
	me.m_pTriIndices = null;
	//me.m_AABB = MathUtils.AABB.create();
	me.m_RootNode = null;
	me.m_Info_TreeDepth = 0;
	me.m_Info_NumNodes = 0;


	let _tempTri = MathUtils.Triangle.create();
	let _tempVec3f = MathUtils.Vec3f.create();

	let getTriangle = function(o_Tri, i_Idx) {
		let pos = me.m_pTriIndices[i_Idx] * triDataStride * 3;
		o_Tri[0] = me.m_pTriangles[pos+0+(triDataStride*0)|0];
		o_Tri[1] = me.m_pTriangles[pos+1+(triDataStride*0)|0];
		o_Tri[2] = me.m_pTriangles[pos+2+(triDataStride*0)|0];
		o_Tri[3] = me.m_pTriangles[pos+0+(triDataStride*1)|0];
		o_Tri[4] = me.m_pTriangles[pos+1+(triDataStride*1)|0];
		o_Tri[5] = me.m_pTriangles[pos+2+(triDataStride*1)|0];
		o_Tri[6] = me.m_pTriangles[pos+0+(triDataStride*2)|0];
		o_Tri[7] = me.m_pTriangles[pos+1+(triDataStride*2)|0];
		o_Tri[8] = me.m_pTriangles[pos+2+(triDataStride*2)|0];
	}

	let computeAABB = function(o_AABB, si, ei) {
		MathUtils.AABB.setMin1f(o_AABB, MathUtils.FLT_MAX);
		MathUtils.AABB.setMax1f(o_AABB, -MathUtils.FLT_MAX);
		for (let i = si; i < ei; i++) {
			getTriangle(_tempTri, i);
			MathUtils.AABB.expandTriangle(o_AABB, _tempTri);
		}

	}

/*
	let BVHNode_MakeLeaf(o_Node, si, ei) {
		o_Node.m_Left = null;
		o_Node.m_Right = null;
		
	}
*/

	let buildTree_Rec = function(io_ParentNode, si, ei, depth) {
		if (depth >	me.m_Info_TreeDepth)	me.m_Info_TreeDepth = depth;

		if (ei - si <= 0) { // hmm; should NEVER happpen...
			logE("BVH buildTree_Rec got invalid list of triangles!");
			io_ParentNode.m_Left = null;
			io_ParentNode.m_Right = null;
			io_ParentNode.m_Leaf_si = -1;
			io_ParentNode.m_Leaf_ei = -1;
			return;
		}

		if (ei - si <= me.m_Param_MaxNumLeafTris || (me.m_Param_MaxTreeDepth >= 0 && depth >= me.m_Param_MaxTreeDepth)) { // leaf node
			io_ParentNode.m_Left = null;
			io_ParentNode.m_Right = null;
			io_ParentNode.m_Leaf_si = si;
			io_ParentNode.m_Leaf_ei = ei;
			return;
		}

		let splitAxis = MathUtils.AABB.getMaxExtendIdx(io_ParentNode.m_AABB);
		MathUtils.AABB.computeCenterVec3f(io_ParentNode.m_AABB, _tempVec3f);
		let splitAxisValue = _tempVec3f[splitAxis];

		//console.log(io_ParentNode.m_AABB);
		//console.log("  splitAxis = " + splitAxis + " value = " + splitAxisValue);

		let left = new BVHNode(me.m_Info_NumNodes);
		let right = new BVHNode(me.m_Info_NumNodes+1);

		me.m_Info_NumNodes += 2;

		let li = si, ri = ei;
		let num = ei - si;

		let m = si + (num/2)|0;

		MathUtils.AABB.reset(left.m_AABB);
		MathUtils.AABB.reset(right.m_AABB);
		
		for (let i = 0; i < num; i++) {
			getTriangle(_tempTri, li);
			MathUtils.Triangle.computeCenterVec3f(_tempTri, _tempVec3f);

			//console.log("");
			//console.log("Tri li=" + li + "; idx = " + me.m_pTriIndices[li] + "; center = " + _tempVec3f);
			//console.log(_tempTri);
			//console.log("   _tempVec3f[splitAxis] = " + _tempVec3f[splitAxis]);

			if (_tempVec3f[splitAxis] < splitAxisValue) { // left
				//console.log("      => left");
				MathUtils.AABB.expandTriangle(left.m_AABB, _tempTri);
				li++;
			} else { // right
				//console.log("      => right");
				MathUtils.AABB.expandTriangle(right.m_AABB, _tempTri);
				ri--;
				// swap:
				let t = me.m_pTriIndices[li]; me.m_pTriIndices[li] = me.m_pTriIndices[ri]; me.m_pTriIndices[ri] = t; 
			}
		}

		

		if ((li != si) && (li != ei)) { // everything went fine, we actually split the array into tw
			m = li;
		} else {
			// recompute AABBs
			//console.log("  BVH: nonoptimal split");
			MathUtils.AABB.reset(left.m_AABB);
			MathUtils.AABB.reset(right.m_AABB);
			computeAABB(left.m_AABB, si, m);
			computeAABB(right.m_AABB, m, ei);
			
			// for now we make a leaf...
			/*io_ParentNode.m_Left = null;
			io_ParentNode.m_Right = null;
			io_ParentNode.m_Leaf_si = si;
			io_ParentNode.m_Leaf_ei = ei;
			return;*/
		}

		//console.log(si + " " + m + " " + ei);
		//console.log("Left box = " + left.m_AABB);
		//console.log("Right box = " + right.m_AABB);
		
		io_ParentNode.m_Left = left;
		io_ParentNode.m_Right = right;

		buildTree_Rec(io_ParentNode.m_Left, si, m, depth+1);
		buildTree_Rec(io_ParentNode.m_Right, m, ei, depth+1);
	}

	me.printTreeRec = function(node, depth) {
		if (node == null) return;
		let str = "";
		for (let i = 0; i < depth; i++) str += "  ";
		str += node.m_NodeIdx + "(" + depth + ") : " + node.m_AABB;
		if (node.m_Leaf_si >= 0) str += " LeafTri = " + node.m_Leaf_si + " to " + node.m_Leaf_ei;
		console.log(str);

		me.printTreeRec(node.m_Left, depth+1);
		me.printTreeRec(node.m_Right, depth+1);
		
	}

	me.build = function(triData) {
		me.m_pTriangles = triData;
		me.m_NumTriangles = (me.m_pTriangles.length / (3 * triDataStride)) | 0;
		me.m_pTriIndices = new Int32Array(me.m_NumTriangles);
		me.m_Info_TreeDepth = 0;
		me.m_RootNode = new BVHNode(0);
		me.m_Info_NumNodes = 1;

		console.log("Building BVH for " + me.m_NumTriangles + " triangles.");

		for (let i = 0; i < me.m_NumTriangles; i++) me.m_pTriIndices[i] = i;

		computeAABB(me.m_RootNode.m_AABB, 0, me.m_NumTriangles);

		buildTree_Rec(me.m_RootNode, 0, me.m_NumTriangles, 0/*startdepth*/);

		console.log("BVH Depth = " + me.m_Info_TreeDepth + "; NumNodes = " + me.m_Info_NumNodes);
		
		//me.printTreeRec(me.m_RootNode, 0);
	};


	let createAndCopyToFlattenedArray_StandardFormat_Rec = function(n, ret) {
		let idx = n.m_NodeIdx;

		ret[idx * 8 + 0] = n.m_AABB[0];
		ret[idx * 8 + 1] = n.m_AABB[1];
		ret[idx * 8 + 2] = n.m_AABB[2];

		ret[idx * 8 + 4] = n.m_AABB[4];
		ret[idx * 8 + 5] = n.m_AABB[5];
		ret[idx * 8 + 6] = n.m_AABB[6];

		if (n.m_Leaf_si >= 0) { // leaf
			ret[idx * 8 + 3] = -n.m_Leaf_si;
			ret[idx * 8 + 7] = -n.m_Leaf_ei;
		} else {
			ret[idx * 8 + 3] = n.m_Left.m_NodeIdx;
			ret[idx * 8 + 7] = n.m_Right.m_NodeIdx;

			createAndCopyToFlattenedArray_StandardFormat_Rec(n.m_Left, ret);
			createAndCopyToFlattenedArray_StandardFormat_Rec(n.m_Right, ret);
		}		
	}

	me.createAndCopyToFlattenedArray_StandardFormat = function() {
		let ret = new Float32Array(me.m_Info_NumNodes * 2 * 4); // per node layout (aabb.minX, aabb.minY, aabb.minZ, >0?leftIdx:-Leaf_si, aabb.max x y z, >0:rightIdx:-Leaf_ei;)

		logI("Creating flattended version of BVH");
		createAndCopyToFlattenedArray_StandardFormat_Rec(me.m_RootNode, ret);

		//console.log(ret);
		return ret;
	}
}

