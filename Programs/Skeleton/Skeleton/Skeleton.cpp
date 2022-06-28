#include "framework.h"

const char* vertexSource = R"(
	#version 330
    precision highp float;

	uniform vec3 wLookAt, wRight, wUp;

	layout(location = 0) in vec2 cCamWindowVertex;
	out vec3 p;

	void main() {
		gl_Position = vec4(cCamWindowVertex, 0, 1);
		p = wLookAt + wRight * cCamWindowVertex.x + wUp * cCamWindowVertex.y;
	}
)";

const char* fragmentSource = R"(
	#version 330
    precision highp float;

	const vec3 La = vec3(0.8f, 0.9f, 0.9f);
	const vec3 Le = vec3(0.4f, 0.4f, 0.4f);
	const vec3 lightPosition = vec3(0.6f, 0.6f, 0.1f);
	const vec3 ka = vec3(0.5f, 0.5f, 0.5f);
	const float shininess = 300.0f;
	const int maxdepth = 6;
	const float epsilon = 0.01f;
	const int faces = 12;

	uniform vec3 wEye, vertices[20];
	uniform int edges[faces*5];
	uniform vec3 kd, ks, F0;

	struct Hit {
		float t;
		vec3 position, normal;
		int material;
	};

	struct Ray {
		vec3 start, dir, weight;
	};

	void getPlane(int i, out vec3 p, out vec3 normal) {
		vec3 p1 = vertices[edges[5*i] - 1], p2 = vertices[edges[5*i+1] - 1], p3 = vertices[edges[5*i+2] - 1];
		normal = cross(p2 - p1, p3 - p1);
		if (dot(p1, normal) < 0) normal = -normal;
		p = p1 + vec3(0, 0, 0.03f);
	}

	Hit intersectDodecahedron(Ray ray, Hit hit, int material) {
		for (int i = 0; i < faces; i++) {
			vec3 p1, normal;
			getPlane(i, p1, normal);
			float ti = abs(dot(normal, ray.dir)) > epsilon ? dot(p1 - ray.start, normal) / dot(normal, ray.dir) : -1;
			if (ti <= epsilon || (ti > hit.t && hit.t > 0)) continue;
			vec3 pintersect = ray.start + ray.dir * ti;
			bool outside = false;
			for (int j = 0; j < faces; j++) {
				if (i == j) continue;
				vec3 p2, n;
				getPlane(j, p2, n);
				if (dot(n, pintersect - p2) > 0) {
					outside = true;
					break;
				}
			}
			if (!outside) {
				vec3 p2;
				bool portal = true;
				for (int j = 0; j < 5; j++) {
					p1 = vertices[edges[5*i + j] - 1];
					p2 = vertices[edges[5*i + ((j+1) % 5)] - 1];
					float t = -1 * dot(p2 - p1, p1 - pintersect) / dot(p2 - p1, p2 - p1);
					float distance = length(p1 + t*(p2-p1) - pintersect);
					if (distance < 0.1 && (ti < hit.t || hit.t < 0)) {
						hit.t = ti;
						hit.position = pintersect;
						hit.normal = normalize(normal);
						hit.material = material;
						portal = false;
						break;
					}
				}
				if (portal) {
					hit.t = ti;
					hit.position = pintersect;
					hit.normal = normalize(normal);
					hit.material = 2;
				}
			}
		}
		return hit;
	}

	vec3 gradf(vec3 r, mat4 Q) {
		vec4 g = vec4(r.x, r.y, r.z, 1) * Q * 2;
		return vec3(g.x, g.y, g.z);
	}

	Hit intersectQuadratic(Ray ray, Hit hit, int material) {
		vec4 S = vec4(ray.start.x, ray.start.y, ray.start.z, 1);
		vec4 D = vec4(ray.dir.x, ray.dir.y, ray.dir.z, 0);
		mat4 Q = mat4(10, 0, 0, 0,
					  0, 10, 0, 0,
					  0, 0, 0, -1,
					  0, 0, -1, 0);

		float a = dot(D * Q, D);
		float b = dot(S * Q, D) * 2;
		float c = dot(S * Q, S);

		float discr = b * b - 4 * a * c;
		if (discr < 0) {
			return hit;
		}
		float t1 = (-b + sqrt(discr)) / (2 * a);
		vec3 p1 = ray.start + t1 * ray.dir;

		float t2 = (-b - sqrt(discr)) / (2 * a);
		vec3 p2 = ray.start + t2 * ray.dir;

		if (length(p1) > 0.3 && length(p2) > 0.3) {
			return hit;
		}
		else if (length(p1) > 0.3 && length(p2) < 0.3) {
			if (t2 > 0 && (t2 < hit.t || hit.t < 0)) {
				hit.t = t2;
				hit.position = p2;
				hit.normal = normalize(gradf(p2, Q));
				hit.material = material;
			}
		}
		else if (length(p2) > 0.3 && length(p1) < 0.3) {
			if (t1 > 0 && (t1 < hit.t || hit.t < 0)) {
				hit.t = t1;
				hit.position = p1;
				hit.normal = normalize(gradf(p1, Q));
				hit.material = material;
			}
		}
		else {
			if (t1 < 0) {
				return hit;
			}
			else if (t2 < 0 && t1 > 0 && (t1 < hit.t || hit.t < 0)) {
				hit.t = t1;
				hit.position = p1;
				hit.normal = normalize(gradf(p1, Q));
				hit.material = material;
			}
			else {
				float t = t1 < t2 ? t1 : t2;
				vec3 p = t1 < t2 ? p1 : p2;
				if (t < hit.t || hit.t < 0) {
					hit.t = t;
					hit.position = p;
					hit.normal = normalize(gradf(p, Q));
					hit.material = material;
				}
			}
		}
		return hit;
	}

	Hit firstIntersect(Ray ray) {
		Hit bestHit;
		bestHit.t = -1;
		bestHit = intersectQuadratic(ray, bestHit, 1);
		bestHit = intersectDodecahedron(ray, bestHit, 0);
		
		if (dot(ray.dir, bestHit.normal) > 0) bestHit.normal = bestHit.normal * (-1);
		return bestHit;
	}

	/*bool shadowIntersect(Ray ray) {	// for directional lights
		Hit bestHit;
		bestHit.t = -1;
		bestHit = intersectQuadratic(ray, bestHit, 1);
		bestHit = intersectDodecahedron(ray, bestHit, 0);
		float lightT = (lightPosition.x - ray.start.x)/ray.dir.x;
		if (bestHit.t > 0 && bestHit.t < lightT) 
			return true;
		return false;
	}*/

	vec3 Rodrigues(vec3 v, vec3 axis, float angle) {
		return v * cos(2*3.14f/(360.0f/angle)) + axis * dot(v, axis) * (1-cos(2*3.14f/(360.0f/angle))) + cross(axis, v) * sin(2*3.14f/(360.0f/angle));
	}

	vec3 trace(Ray ray) {
		vec3 outRadiance = vec3(0, 0, 0);
		for(int d = 0; d < maxdepth; d++) {
			Hit hit = firstIntersect(ray);
			if (hit.t < 0) break;
			if (hit.material == 0) {
				vec3 lightDir = normalize(lightPosition - hit.position);
				float cosTheta = dot(hit.normal, lightDir);

				/*Ray shadowRay;
				shadowRay.start = hit.position + hit.normal * epsilon;
				shadowRay.dir = lightPosition - shadowRay.start;
				if (cosTheta > 0 && !shadowIntersect(shadowRay)) {*/


				if (cosTheta > 0) {
					vec3 LeIn = Le / dot(lightPosition - hit.position, lightPosition - hit.position);
					outRadiance += ray.weight * LeIn * kd * cosTheta;
					vec3 halfway = normalize(-ray.dir + lightDir);
					float cosDelta = dot(hit.normal, halfway);
					if (cosDelta > 0) outRadiance += ray.weight * LeIn * ks * pow(cosDelta, shininess);
				}
				ray.weight *= ka;
				break;
			}
			else if (hit.material == 1) {
				ray.weight *= F0 + (vec3(1, 1, 1) - F0) * pow(dot(-ray.dir, hit.normal), 5);
				ray.start = hit.position + hit.normal * epsilon;
				ray.dir = reflect(ray.dir, hit.normal);
			}
			else {
				ray.start = Rodrigues(hit.position + hit.normal * epsilon, -hit.normal, 72);
				vec3 newDir = Rodrigues(ray.dir, -hit.normal, 72);
				ray.dir = reflect(newDir, hit.normal);
			}
		}
		outRadiance += ray.weight * La;
		return outRadiance;
	}

	in  vec3 p;
	out vec4 fragmentColor;

	void main() {
		Ray ray;
		ray.start = wEye; 
		ray.dir = normalize(p - wEye);
		ray.weight = vec3(1, 1, 1);
		fragmentColor = vec4(trace(ray), 1); 
	}
)";

struct Camera {
	vec3 eye, lookat, right, pvup, rvup;
	float fov = 50 * (float)M_PI / 180;
public:
	Camera() : eye(0, 1, 0.4), pvup(0, 0, 1), lookat(0, 0, 0.15) {
		set();
	}
	void set() {
		vec3 w = eye - lookat;
		float focus = length(w);
		right = normalize(cross(pvup, w)) * focus * tanf(fov / 2);
		rvup = normalize(cross(w, right)) * focus * tanf(fov / 2);
	}
	void Animate(float dt) {
		float r = sqrtf(eye.x * eye.x + eye.y * eye.y);
		eye = vec3(r * cos(dt) + lookat.x, r * sin(dt) + lookat.y, eye.z);
		set();
	}
	/*void Animate(float dt) {   // r�gi
		eye = vec3((eye.x - lookat.x) * cos(dt) + (eye.z - lookat.z) * sin(dt) + lookat.x,
			eye.y,
			-(eye.x - lookat.x) * sin(dt) + (eye.z - lookat.z) * cos(dt) + lookat.z);
		set();
	}*/
	/*void Animate(float dt) {   r�gibb
		vec3 d = eye - lookat;
		eye = vec3(d.x * cos(dt) + d.z * sin(dt), d.y, -d.x * sin(dt) + d.z * cos(dt)) + lookat;
		set(eye, lookat, up, fov);
	}*/
};

GPUProgram gpuProgram;
Camera camera;

float F(float n, float kappa) {
	return ((n - 1) * (n - 1) + kappa * kappa) / ((n + 1) * (n + 1) + kappa * kappa);
}

void onInitialization() {
	glViewport(0, 0, windowWidth, windowHeight);

	unsigned int vao, vbo;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	
	float vertexCoords[] = { -1, -1,  1, -1,  1, 1,  -1, 1 };
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexCoords), vertexCoords, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, NULL);
	
	gpuProgram.create(vertexSource, fragmentSource, "fragmentColor");

	const float fi = 1.618f;
	const float fiRecip = 0.618f;

	std::vector<vec3> vertices = { vec3(0, fiRecip, fi), vec3(0, -fiRecip, fi), vec3(0, -fiRecip, -fi), vec3(0, fiRecip, -fi),
									vec3(fi, 0, fiRecip), vec3(-fi, 0, fiRecip), vec3(-fi, 0, -fiRecip), vec3(fi, 0, -fiRecip),
									vec3(fiRecip, fi, 0), vec3(-fiRecip, fi, 0), vec3(-fiRecip, -fi, 0), vec3(fiRecip, -fi, 0),
									vec3(1, 1, 1), vec3(-1, 1, 1), vec3(-1, -1, 1), vec3(1, -1, 1),
									vec3(1, -1, -1), vec3(1, 1, -1), vec3(-1, 1, -1), vec3(-1, -1, -1) };
	for (int i = 0; i < vertices.size(); i++) {
		gpuProgram.setUniform(vertices[i], "vertices[" + std::to_string(i) + "]");
	}

	std::vector<int> edges = { 1, 2, 16, 5, 13,
							   1, 13, 9, 10, 14,
							   1, 14, 6, 15, 2,
							   2, 15, 11, 12, 16,
						       3, 4, 18, 8, 17,
						       3, 17, 12, 11, 20,
							   3, 20, 7, 19, 4,
							   19, 10, 9, 18, 4,
							   16, 12, 17, 8, 5,
							   5, 8, 18, 9, 13,
							   14, 10, 19, 7, 6,
							   6, 7, 20, 11, 15 };
	for (int i = 0; i < edges.size(); i++) {
		gpuProgram.setUniform(edges[i], "edges[" + std::to_string(i) + "]");
	}

	gpuProgram.setUniform(vec3(0.1f, 1.0f, 1.0f), "kd");
	gpuProgram.setUniform(vec3(5, 5, 5), "ks");
	gpuProgram.setUniform(vec3(F(0.17, 3.1), F(0.35, 2.7), F(1.5, 1.9)), "F0");

}

void onDisplay() {
	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	gpuProgram.setUniform(camera.eye, "wEye");
	gpuProgram.setUniform(camera.lookat, "wLookAt");
	gpuProgram.setUniform(camera.right, "wRight");
	gpuProgram.setUniform(camera.rvup, "wUp");
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	glutSwapBuffers();
}

bool animate = true;

void onKeyboard(unsigned char key, int pX, int pY) {
	if (key == ' ') {
		animate = !animate;
	}
	else if (key == 'w') {
		camera.eye = camera.eye + 0.05 * normalize(camera.lookat - camera.eye);
	}
	else if (key == 's') {
		camera.eye = camera.eye - 0.05 * normalize(camera.lookat - camera.eye);
	}
	else if (key == 'a') {
		camera.eye = camera.eye - 0.05 * normalize(camera.lookat - camera.eye);
	}
	else if (key == 'd') {
		camera.eye = camera.eye + 0.05 * normalize(camera.lookat - camera.eye);
	}
	else if (key == '4') {
		camera.lookat = camera.lookat + 0.05 * normalize(cross(camera.rvup, camera.lookat - camera.eye));
	}
	else if (key == '6') {
		camera.lookat = camera.lookat - 0.05 * normalize(cross(camera.rvup, camera.lookat - camera.eye));
	}
	else if (key == '8') {
		camera.lookat = camera.lookat + 0.05 * camera.rvup;
	}
	else if (key == '2') {
		camera.lookat = camera.lookat - 0.05 * camera.rvup;
	}
	camera.set();     // kommentezd ki ezt a sort, es akkor a "w" meg az "s" mokas
	glutPostRedisplay();
}

void onKeyboardUp(unsigned char key, int pX, int pY) {
}

void onMouseMotion(int pX, int pY) {
}

void onMouse(int button, int state, int pX, int pY) {
}

void onIdle() {
	if (animate) {
		long time = glutGet(GLUT_ELAPSED_TIME);
		camera.Animate(time / 3000.0f);
		glutPostRedisplay();
	}
}