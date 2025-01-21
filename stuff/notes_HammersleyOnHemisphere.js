(function(){function r(a){a=(a<<16|a>>>16)>>>0;a=((a&1431655765)<<1|(a&2863311530)>>>1)>>>0;a=((a&858993459)<<2|(a&3435973836)>>>2)>>>0;a=((a&252645135)<<4|(a&4042322160)>>>4)>>>0;return(((a&16711935)<<8|(a&4278255360)>>>8)>>>0)/4294967296}(new function(){function a(){var a=e.getContext("2d"),i=e.width,j=e.height,l,h,b,c,f,d;e.width=e.width;h=i/32;b=j-j/32;l=i-2*h;j-=2*b;if(c=parseInt(g.value,10)){c=Math.min(c,2048);if(64>=c){a.strokeStyle="lightgray";a.beginPath();for(f=1;f<c;f+=1)a.moveTo(h+f/c*
l,b),a.lineTo(h+f/c*l,b+j),a.moveTo(h,b+f/c*j),a.lineTo(h+l,b+f/c*j);a.stroke()}f=i/64;a.strokeStyle="black";a.beginPath();a.moveTo(h-f,b);a.lineTo(h+l,b);a.moveTo(h,b+f);a.lineTo(h,b+j);a.stroke();d=i/(16*Math.sqrt(c));1>d&&(d=1);a.fillStyle="#0088CC";for(f=0;f<c;f+=1)i=[f/c,r(f)],i[0]=i[0]*l+h,i[1]=i[1]*j+b,a.beginPath(),a.arc(i[0],i[1],d,0,2*Math.PI,!1),a.fill()}}var e,g;this.init=function(){e=document.getElementById("canvas-Hammersley2d");g=document.getElementById("input-Hammersley2d-numPoints");
g.onchange=a;a()}}).init();(new function(){var a,e,g;function s(){d.position.x=3*Math.cos(d.rotPhi)*Math.sin(d.rotTheta);d.position.y=3*Math.cos(d.rotTheta);d.position.z=3*Math.sin(d.rotPhi)*Math.sin(d.rotTheta);d.lookAt(new THREE.Vector3(0,0,0));c.render(n,d)}function i(m){var q=m.pageY-b.offsetTop;a=m.pageX-b.offsetLeft;e=q;g=!0}function j(){g=!1}function l(m){var q=m.pageX-b.offsetLeft,m=m.pageY-b.offsetTop,f,c;g?(f=a-q,c=e-m,d.rotTheta+=c/64,d.rotPhi-=f/64,0.01>d.rotTheta&&(d.rotTheta=0.01),d.rotTheta>
Math.PI/2.5&&(d.rotTheta=Math.PI/2.5),a=q,e=m,s()):g=!1}function h(){var a,b,d,c,h,i,j;for(a in p)n.remove(p[a]);h=document.getElementById("input-HammersleyOnSphere-mapping").selectedIndex;if(d=parseInt(f.value,10)){d=Math.min(d,2048);new THREE.Geometry;j=new THREE.MeshBasicMaterial({color:16711680});i=Math.max(0.02,0.125/Math.sqrt(d));for(a=0;a<d;a+=1){b=[a/d,r(a)];if(0===h){var e=c=void 0,g=void 0;c=2*b[1]*Math.PI;e=1-b[0];g=Math.sqrt(1-e*e);c=[Math.cos(c)*g,Math.sin(c)*g,e]}else 1===h&&(g=e=c=
void 0,c=2*b[1]*Math.PI,e=Math.sqrt(1-b[0]),g=Math.sqrt(1-e*e),c=[Math.cos(c)*g,Math.sin(c)*g,e]);b=new THREE.Mesh(new THREE.SphereGeometry(i,4,4),j);b.position.x=c[0];b.position.y=c[2];b.position.z=c[1];n.add(b);p.push(b)}s()}}var b,c,f,d,n,p=[];g=e=a=void 0;var k;this.init=function(){var a,e,g,p=!0;b=document.getElementById("canvas-HammersleyOnHemisphere");window.WebGLRenderingContext&&(c=new THREE.WebGLRenderer({canvas:b}));if(!c&&(p=!1,c=new THREE.CanvasRenderer({canvas:b}),!c))return;c.setSize(b.width,
b.height);b.addEventListener("mousedown",i,!1);b.addEventListener("mouseup",j,!1);b.addEventListener("mouseout",j,!1);b.addEventListener("mousemove",l,!1);console.log(c);d=new THREE.PerspectiveCamera(45,b.width/b.height,0.1,16);d.rotTheta=Math.PI/4;d.rotPhi=Math.PI/2;n=new THREE.Scene;k=p?new THREE.Mesh(new THREE.SphereGeometry(1,32,16),new THREE.MeshLambertMaterial({color:35020})):new THREE.Mesh(new THREE.SphereGeometry(1,24,12),new THREE.MeshLambertMaterial({color:35020,wireframe:!0,side:THREE.DoubleSide}));
g=[];for(a=0;a<k.geometry.faces.length;a+=1)e=k.geometry.faces[a],e.a&&0>k.geometry.vertices[e.a].y||e.b&&0>k.geometry.vertices[e.b].y||e.c&&0>k.geometry.vertices[e.c].y||e.d&&0>k.geometry.vertices[e.d].y||g.push(e);k.geometry.faces=g;n.add(k);p&&(a=new THREE.Mesh(new THREE.PlaneGeometry(4,4),new THREE.MeshBasicMaterial({color:8421504})),a.rotation.x=-Math.PI/2,n.add(a));a=new THREE.DirectionalLight(16777215);a.position.set(1,1,1).normalize();n.add(a);f=document.getElementById("input-HammersleyOnSphere-numPoints");
f.onchange=h;document.getElementById("input-HammersleyOnSphere-mapping").onchange=h;h()}}).init()})();
