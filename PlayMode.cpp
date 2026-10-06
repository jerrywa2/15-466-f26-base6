#include "PlayMode.hpp"

#include "DrawLines.hpp"
#include "gl_errors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

//DONT FUSE MULTIPLY-ADD
#if defined(_MSC_VER)
#pragma fp_contract(off)
#elif defined(__clang__)
#pragma STDC FP_CONTRACT OFF
#else
#pragma GCC optimize("fp-contract=off")
#endif

//------ tuning -----

constexpr float Tick = 1.0f / 60.0f; //fixed simulation step
constexpr uint32_t Substeps = 8; //? test later
constexpr float H = Tick / float(Substeps); //substep length

constexpr float Gravity = 20.0f;
constexpr float Restitution = 0.7f; //bounciness
constexpr float Friction = 0.2f;
constexpr float Drag = 0.05f; //fraction of velocity lost per second --------------------- test later

constexpr uint32_t RunTicks = 30 * 60; //run lasts 30 seconds from luanch
constexpr float PlayerRadius = 0.4f;
constexpr float LaunchScale = 2.5f; //launch speed per unit of distance from ball to mouse
constexpr float MaxLaunchSpeed = 24.0f;

//two maps are sawteeth:each tooth ramps gently down to the right, then jumps back up
constexpr float KnotSpacing = 1.5f; //horizontal distance between terrain points
constexpr int32_t ToothKnots = 6; //knots per tooth
constexpr float ToothMinHeight = 0.6f;
constexpr float ToothMaxHeight = 1.8f;
constexpr float CeilingY = 12.0f; //ceiling map hangs down from here; floor map rises up from 0
constexpr int32_t FlatKnots = 4; //knots before this one are flat (the launch pad)

//boulder recycle
constexpr float BoulderBehind = 22.0f;
constexpr float BoulderAhead = 24.0f;
constexpr float BoulderSpread = 14.0f;

//camera
constexpr float ViewHalfHeight = 8.0f;
constexpr float CameraLead = 5.0f; //camera look-ahead

//--------- helpers -----

//hash helper
static float hash01(uint32_t x) {
	x ^= x >> 16; x *= 0x7feb352du;
	x ^= x >> 15; x *= 0x846ca68bu;
	x ^= x >> 16;
	return float(x >> 8) * (1.0f / 16777216.0f);
}

//terrain knots:
static float tooth_height(int32_t i, uint32_t seed) {
	if (i < FlatKnots) return 0.0f;
	uint32_t tooth = uint32_t(i / ToothKnots);
	int32_t step = i % ToothKnots;
	float peak = ToothMinHeight + (ToothMaxHeight - ToothMinHeight) * hash01(tooth * 2u + seed);
	return peak * float(ToothKnots - 1 - step) / float(ToothKnots - 1);
}
static glm::vec2 floor_knot(int32_t i) {
	return glm::vec2(float(i) * KnotSpacing, tooth_height(i, 0u));
}
static glm::vec2 ceiling_knot(int32_t i) {
	//ceiling teeth are shifted over by half a tooth
	return glm::vec2(float(i) * KnotSpacing, CeilingY - tooth_height(i + ToothKnots / 2, 1u));
}

static float length(glm::vec2 v) {
	return std::sqrt(v.x * v.x + v.y * v.y);
}
static float dot(glm::vec2 a, glm::vec2 b) {
	return a.x * b.x + a.y * b.y;
}
static float cross(glm::vec2 a, glm::vec2 b) {
	return a.x * b.y - a.y * b.x;
}

//contact
constexpr uint32_t World = 0xffffffffu;
struct Contact {
	uint32_t a = 0;
	uint32_t b = World;
	glm::vec2 point = glm::vec2(0.0f); //if b == world: the point on the world being touched
	glm::vec2 normal = glm::vec2(0.0f, 1.0f); //unit vector pointing from b toward a
	float approach = 0.0f; //normal velocity before the position solve (negative = moving together)
	float lambda = 0.0f; //total push applied by the position solve
};

//terrain contact
static void collide_segment(uint32_t index, PlayMode::Body const &body, glm::vec2 from, glm::vec2 to, glm::vec2 outside, std::vector< Contact > *contacts) {
	glm::vec2 along = to - from;
	float t = dot(body.pos - from, along) / dot(along, along);
	t = std::max(0.0f, std::min(1.0f, t));
	glm::vec2 closest = from + t * along;
	glm::vec2 to_center = body.pos - closest;

	Contact contact;
	contact.a = index;
	contact.point = closest;
	if (dot(to_center, outside) <= 0.0f) {
		//center is inside the ground; push straight out of the surface:
		contact.normal = outside / length(outside);
	} else {
		float dist = length(to_center);
		if (dist >= body.radius) return; //not touching
		contact.normal = to_center / dist;
	}
	contact.approach = dot(contact.normal, body.vel);
	contacts->emplace_back(contact);
}

//place boulders
static void place_boulder(PlayMode::State *state, uint32_t index, float x) {
	uint32_t seed = state->spawn_count * 3u + 1000u;
	state->spawn_count += 1;

	PlayMode::Body &body = state->bodies[index];
	body.radius = 0.3f + 0.6f * hash01(seed);
	body.inv_mass = 1.0f / (body.radius * body.radius); //mass proportional to area
	body.pos.x = x + BoulderSpread * hash01(seed + 1u);
	body.pos.y = ToothMaxHeight + 1.0f + (CeilingY - 2.0f * ToothMaxHeight - 2.0f) * hash01(seed + 2u);
	body.vel = glm::vec2(0.0f);
	body.angle = 0.0f;
	body.omega = 0.0f;
}

//----- physics --------

void PlayMode::reset() {
	state = State();
	state.gravity = -Gravity;
	state.time_left = RunTicks;

	Body &player = state.bodies[0];
	player.radius = PlayerRadius;
	player.inv_mass = 1.0f / (player.radius * player.radius);
	player.pos = glm::vec2(2.0f, player.radius);

	for (uint32_t i = 1; i < BodyCount; ++i) {
		//start boulders:
		place_boulder(&state, i, 12.0f + 2.0f * float(i));
	}

	pending = Input();
	accumulator = 0.0f;

	history.clear();
	inputs.clear();
}

void PlayMode::step(Input const &input) {
	if (state.time_left == 0) return; //run over

	Body &player = state.bodies[0];

	//input
	if (input.launch && !state.launched) {
		player.vel = input.launch_vel;
		state.launched = 1;
	}
	if (input.flip && state.launched) {
		state.gravity = -state.gravity;
	}

	static std::vector< Contact > contacts; //(static just to avoid reallocating every tick
	std::array< glm::vec2, BodyCount > prev_pos;

	for (uint32_t substep = 0; substep < Substeps; ++substep) {
		//integrate:
		for (uint32_t i = 0; i < BodyCount; ++i) {
			Body &body = state.bodies[i];
			prev_pos[i] = body.pos;
			body.vel.y += H * state.gravity;
			body.vel -= body.vel * (Drag * H);
			body.omega -= body.omega * (Drag * H);
			body.pos += H * body.vel;
			body.angle += H * body.omega;
			//bound angle
			if (body.angle > 4.0f) body.angle -= 8.0f;
			if (body.angle < -4.0f) body.angle += 8.0f;
		}

		//contacts:
		contacts.clear();
		for (uint32_t i = 0; i < BodyCount; ++i) {
			Body const &body = state.bodies[i];

			//left wall:
			if (body.pos.x < body.radius) {
				Contact contact;
				contact.a = i;
				contact.point = glm::vec2(0.0f, body.pos.y);
				contact.normal = glm::vec2(1.0f, 0.0f);
				contact.approach = body.vel.x;
				contacts.emplace_back(contact);
			}

			//terrain:
			int32_t first = int32_t(std::floor((body.pos.x - body.radius) / KnotSpacing));
			int32_t last = int32_t(std::floor((body.pos.x + body.radius) / KnotSpacing));
			for (int32_t k = first; k <= last; ++k) {
				{ //floor:
					glm::vec2 from = floor_knot(k);
					glm::vec2 to = floor_knot(k + 1);
					glm::vec2 up = glm::vec2(-(to.y - from.y), to.x - from.x);
					collide_segment(i, body, from, to, up, &contacts);
				}
				{ //ceiling:
					glm::vec2 from = ceiling_knot(k);
					glm::vec2 to = ceiling_knot(k + 1);
					glm::vec2 down = glm::vec2(to.y - from.y, -(to.x - from.x));
					collide_segment(i, body, from, to, down, &contacts);
				}
			}

			//bodies
			for (uint32_t j = 0; j < i; ++j) {
				Body const &other = state.bodies[j];
				glm::vec2 between = body.pos - other.pos;
				float dist = length(between);
				if (dist >= body.radius + other.radius) continue;
				Contact contact;
				contact.a = i;
				contact.b = j;
				contact.normal = (dist > 0.0f ? between / dist : glm::vec2(0.0f, 1.0f));
				contact.approach = dot(contact.normal, body.vel - other.vel);
				contacts.emplace_back(contact);
			}
		}

		//separate ballz
		for (Contact &contact : contacts) {
			Body &a = state.bodies[contact.a];
			if (contact.b == World) {
				float depth = a.radius - dot(a.pos - contact.point, contact.normal);
				if (depth <= 0.0f) continue;
				a.pos += depth * contact.normal;
				contact.lambda += depth / a.inv_mass;
			} else {
				Body &b = state.bodies[contact.b];
				float depth = a.radius + b.radius - dot(a.pos - b.pos, contact.normal);
				if (depth <= 0.0f) continue;
				float lambda = depth / (a.inv_mass + b.inv_mass);
				a.pos += (lambda * a.inv_mass) * contact.normal;
				b.pos -= (lambda * b.inv_mass) * contact.normal;
				contact.lambda += lambda;
			}
		}

		//update velocities
		for (uint32_t i = 0; i < BodyCount; ++i) {
			state.bodies[i].vel = (state.bodies[i].pos - prev_pos[i]) / H;
		}

		//solve velocity
		for (Contact const &contact : contacts) {
			if (contact.lambda <= 0.0f) continue; //no contact

			//world body
			Body world;
			world.inv_mass = 0.0f;
			world.radius = 0.0f;
			Body &a = state.bodies[contact.a];
			Body &b = (contact.b == World ? world : state.bodies[contact.b]);

			glm::vec2 n = contact.normal;

			//contact offsets
			glm::vec2 ra = -a.radius * n;
			glm::vec2 rb = b.radius * n;

			float a_inv_inertia = 2.0f * a.inv_mass / (a.radius * a.radius);
			float b_inv_inertia = (contact.b == World ? 0.0f : 2.0f * b.inv_mass / (b.radius * b.radius));

			//contact velocity:
			glm::vec2 va = a.vel + a.omega * glm::vec2(-ra.y, ra.x);
			glm::vec2 vb = b.vel + b.omega * glm::vec2(-rb.y, rb.x);
			glm::vec2 v = va - vb;
			float vn = dot(n, v);
			glm::vec2 vt = v - vn * n;

			{ //friction:
				float speed = length(vt);
				if (speed > 0.0f) {
					float w = a.inv_mass + b.inv_mass + a.radius * a.radius * a_inv_inertia + b.radius * b.radius * b_inv_inertia;
					float amount = std::min(Friction * contact.lambda / H, speed / w);
					glm::vec2 impulse = -(amount / speed) * vt;
					a.vel += impulse * a.inv_mass;
					a.omega += cross(ra, impulse) * a_inv_inertia;
					b.vel -= impulse * b.inv_mass;
					b.omega -= cross(rb, impulse) * b_inv_inertia;
				}
			}

			float bounce = Restitution;
			if (-contact.approach <= 2.0f * Gravity * H) bounce = 0.0f; //stop small bounces
			float want_vn = std::max(-bounce * contact.approach, 0.0f);
			glm::vec2 impulse = ((want_vn - vn) / (a.inv_mass + b.inv_mass)) * n;
			a.vel += impulse * a.inv_mass;
			b.vel -= impulse * b.inv_mass;
		}
	}

	//recycle boulders:
	for (uint32_t i = 1; i < BodyCount; ++i) {
		if (state.bodies[i].pos.x < player.pos.x - BoulderBehind) {
			place_boulder(&state, i, player.pos.x + BoulderAhead);
		}
	}

	if (state.launched) state.time_left -= 1;
	state.tick += 1;
}

//state hash
uint32_t PlayMode::hash_state(State const &to_hash) {

	uint32_t hash = 2166136261u;
	auto add_int = [&hash](uint32_t value) {
		for (uint32_t byte = 0; byte < 4; ++byte) {
			hash ^= (value >> (8 * byte)) & 0xffu;
			hash *= 16777619u;
		}
	};
	auto add_float = [&add_int](float value) {
		uint32_t bits;
		std::memcpy(&bits, &value, 4);
		add_int(bits);
	};
	for (Body const &body : to_hash.bodies) {
		add_float(body.pos.x);
		add_float(body.pos.y);
		add_float(body.vel.x);
		add_float(body.vel.y);
		add_float(body.angle);
		add_float(body.omega);
		add_float(body.radius);
		add_float(body.inv_mass);
	}
	add_float(to_hash.gravity);
	add_int(to_hash.time_left);
	add_int(to_hash.launched);
	add_int(to_hash.spawn_count);
	add_int(to_hash.tick);
	return hash;
}

//---- determinism ----

//test inputs
constexpr uint32_t DemoHash = 0xC46E4DEEu;
static std::vector< PlayMode::Input > demo_inputs() {
	//demo inputs:
	std::vector< PlayMode::Input > demo(30 + RunTicks);
	demo[30].launch = 1;
	demo[30].launch_vel = glm::vec2(20.0f, 8.0f);
	for (uint32_t tick = 80; tick < demo.size(); tick += 50) {
		demo[tick].flip = 1;
	}
	return demo;
}

void PlayMode::start_replay(std::vector< Input > const &replay_inputs, uint32_t expected_hash) {
	replay = replay_inputs;
	replay_expected_hash = expected_hash;
	replay_message = "";
	replaying = true;
	paused = false;
	reset();
}

//----- game -------

PlayMode::PlayMode() {
	reset();
}

PlayMode::~PlayMode() {
}

glm::vec2 PlayMode::camera_center() const {
	return glm::vec2(state.bodies[0].pos.x + CameraLead, 0.5f * CeilingY);
}

glm::vec2 PlayMode::launch_velocity() const {
	glm::vec2 vel = LaunchScale * (mouse_world - state.bodies[0].pos);
	float speed = length(vel);
	if (speed > MaxLaunchSpeed) vel *= MaxLaunchSpeed / speed;
	return vel;
}

bool PlayMode::handle_event(SDL_Event const &evt, glm::uvec2 const &window_size) {
	if (evt.type == SDL_EVENT_KEY_DOWN) {
		if (evt.key.key == SDLK_SPACE) {
			if (!evt.key.repeat) pending.flip = 1;
			return true;
		} else if (evt.key.key == SDLK_R) {
			replaying = false;
			replay_message = "";
			paused = false;
			reset();
			return true;
		} else if (evt.key.key == SDLK_P) {
			paused = !paused;
			return true;
		} else if (evt.key.key == SDLK_BACKSPACE) {
			rewind_held = true;
			return true;
		} else if (evt.key.key == SDLK_V) {
			//verify replay
			start_replay(inputs, hash_state(state));
			return true;
		} else if (evt.key.key == SDLK_D) {
			start_replay(demo_inputs(), DemoHash);
			return true;
		}
	} else if (evt.type == SDL_EVENT_KEY_UP) {
		if (evt.key.key == SDLK_BACKSPACE) {
			rewind_held = false;
			return true;
		}
	} else if (evt.type == SDL_EVENT_MOUSE_MOTION || evt.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
		//mouse position
		float x = (evt.type == SDL_EVENT_MOUSE_MOTION ? evt.motion.x : evt.button.x);
		float y = (evt.type == SDL_EVENT_MOUSE_MOTION ? evt.motion.y : evt.button.y);
		float aspect = float(window_size.x) / float(window_size.y);
		glm::vec2 clip = glm::vec2(
			2.0f * x / float(window_size.x) - 1.0f,
			1.0f - 2.0f * y / float(window_size.y)
		);
		mouse_world = camera_center() + ViewHalfHeight * glm::vec2(clip.x * aspect, clip.y);

		if (evt.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
			pending.launch = 1;
			pending.launch_vel = launch_velocity();
		}
		return true;
	}

	return false;
}

void PlayMode::update(float elapsed) {
	//fixed ticks
	accumulator += elapsed;
	while (accumulator >= Tick) {
		accumulator -= Tick;

		if (rewind_held) {
			replaying = false;
			replay_message = "";
			for (uint32_t i = 0; i < 2; ++i) { //twice as fast as normal
				if (history.empty()) break;
				state = history.back();
				history.pop_back();
				inputs.pop_back();
			}
			pending = Input();
			continue;
		}
  
		if (paused) continue;
		if (state.time_left == 0) continue; //run is over

		Input input = pending;
		pending = Input();
		if (replaying) input = replay[state.tick];

		history.emplace_back(state);
		inputs.emplace_back(input);
		step(input);

		best_distance = std::max(best_distance, state.bodies[0].pos.x);

		if (replaying && state.tick == replay.size()) {
			//replay done
			replaying = false;
			paused = true;
			char hex[16];
			std::snprintf(hex, sizeof(hex), "%08X", replay_expected_hash);
			if (hash_state(state) == replay_expected_hash) {
				replay_message = "REPLAY MATCHED (expected " + std::string(hex) + ")";
			} else {
				replay_message = "REPLAY DID NOT MATCH (expected " + std::string(hex) + ")";
			}
		}
	}
}

void PlayMode::draw(glm::uvec2 const &drawable_size) {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);

	float aspect = float(drawable_size.x) / float(drawable_size.y);
	glm::vec2 center = camera_center();
	Body const &player = state.bodies[0];

	{ //world
		float sx = 1.0f / (ViewHalfHeight * aspect);
		float sy = 1.0f / ViewHalfHeight;
		DrawLines lines(glm::mat4(
			sx, 0.0f, 0.0f, 0.0f,
			0.0f, sy, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			-center.x * sx, -center.y * sy, 0.0f, 1.0f
		));

		//active side
		glm::u8vec4 floor_color = (state.gravity < 0.0f ? glm::u8vec4(0xff, 0xcc, 0x66, 0xff) : glm::u8vec4(0x77, 0x66, 0x44, 0xff));
		glm::u8vec4 ceiling_color = (state.gravity > 0.0f ? glm::u8vec4(0x66, 0xcc, 0xff, 0xff) : glm::u8vec4(0x44, 0x66, 0x77, 0xff));

		//terrain
		int32_t first = int32_t(std::floor((center.x - ViewHalfHeight * aspect) / KnotSpacing));
		int32_t last = int32_t(std::floor((center.x + ViewHalfHeight * aspect) / KnotSpacing));
		for (int32_t k = first; k <= last; ++k) {
			glm::vec2 a = floor_knot(k);
			glm::vec2 b = floor_knot(k + 1);
			lines.draw(glm::vec3(a, 0.0f), glm::vec3(b, 0.0f), floor_color);
			lines.draw(glm::vec3(a.x, a.y, 0.0f), glm::vec3(a.x, -10.0f, 0.0f), floor_color);

			a = ceiling_knot(k);
			b = ceiling_knot(k + 1);
			lines.draw(glm::vec3(a, 0.0f), glm::vec3(b, 0.0f), ceiling_color);
			lines.draw(glm::vec3(a.x, a.y, 0.0f), glm::vec3(a.x, CeilingY + 10.0f, 0.0f), ceiling_color);

			//distance:
			if (k % 10 == 0 && k >= 0) {
				lines.draw_text(std::to_string(int32_t(a.x)),
					glm::vec3(a.x + 0.2f, -0.9f, 0.0f),
					glm::vec3(0.6f, 0.0f, 0.0f), glm::vec3(0.0f, 0.6f, 0.0f),
					glm::u8vec4(0x88, 0x88, 0x88, 0xff));
			}
		}
		//left wall
		lines.draw(glm::vec3(0.0f, -10.0f, 0.0f), glm::vec3(0.0f, CeilingY + 10.0f, 0.0f), glm::u8vec4(0xff));

		//ballz
		for (uint32_t i = 0; i < BodyCount; ++i) {
			Body const &body = state.bodies[i];
			glm::u8vec4 color = (i == 0 ? glm::u8vec4(0xff, 0x44, 0x88, 0xff) : glm::u8vec4(0xaa, 0xaa, 0xaa, 0xff));
			constexpr uint32_t Sides = 24;
			for (uint32_t s = 0; s < Sides; ++s) {
				float a0 = body.angle + float(s) / float(Sides) * 2.0f * float(M_PI);
				float a1 = body.angle + float(s + 1) / float(Sides) * 2.0f * float(M_PI);
				lines.draw(
					glm::vec3(body.pos + body.radius * glm::vec2(std::cos(a0), std::sin(a0)), 0.0f),
					glm::vec3(body.pos + body.radius * glm::vec2(std::cos(a1), std::sin(a1)), 0.0f),
					color);
			}
			lines.draw(
				glm::vec3(body.pos, 0.0f),
				glm::vec3(body.pos + body.radius * glm::vec2(std::cos(body.angle), std::sin(body.angle)), 0.0f),
				color);
		}

		//aim:
		if (!state.launched) {
			lines.draw(glm::vec3(player.pos, 0.0f), glm::vec3(player.pos + 0.2f * launch_velocity(), 0.0f), glm::u8vec4(0xff, 0x44, 0x88, 0xff));
		}
	}

	{ //text
		DrawLines lines(glm::mat4(
			1.0f / aspect, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		));
		constexpr float Size = 0.07f;
		auto text = [&](std::string const &str, float line) {
			lines.draw_text(str,
				glm::vec3(-aspect + 0.5f * Size, 1.0f - (1.5f + 1.3f * line) * Size, 0.0f),
				glm::vec3(Size, 0.0f, 0.0f), glm::vec3(0.0f, Size, 0.0f),
				glm::u8vec4(0xff, 0xff, 0xff, 0xff));
		};

		text("Distance " + std::to_string(int32_t(player.pos.x))
			+ "   Best " + std::to_string(int32_t(best_distance))
			+ "   Time " + std::to_string(state.time_left / 60) + "." + std::to_string(state.time_left % 60 / 6)
			+ "   Gravity " + (state.gravity < 0.0f ? "down" : "up"), 0.0f);

		std::string message;
		if (replay_message != "") message = replay_message;
		else if (replaying) message = "Replaying recorded inputs...";
		else if (rewind_held) message = "Rewinding...";
		else if (paused) message = "Paused - P continues";
		else if (!state.launched) message = "Aim with the mouse, click to shoot";
		else if (state.time_left > 0) message = "Space flips gravity - Backspace to rewind if you get knocked back!";
		else message = "Time is up - R restarts, hold Backspace to rewind";
		text(message, 1.0f);

		char hex[16];
		std::snprintf(hex, sizeof(hex), "%08X", hash_state(state));
		lines.draw_text("Space flip   Backspace rewind   R restart   P pause   V verify replay   D determinism demo     state " + std::string(hex),
			glm::vec3(-aspect + 0.5f * Size, -1.0f + 0.5f * Size, 0.0f),
			glm::vec3(0.7f * Size, 0.0f, 0.0f), glm::vec3(0.0f, 0.7f * Size, 0.0f),
			glm::u8vec4(0x88, 0x88, 0x88, 0xff));
	}

	GL_ERRORS();
}