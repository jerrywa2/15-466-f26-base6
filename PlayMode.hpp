#include "Mode.hpp"

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct PlayMode : Mode {
    PlayMode();
    virtual ~PlayMode();

    //main loop:
    virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
    virtual void update(float elapsed) override;
    virtual void draw(glm::uvec2 const &drawable_size) override;

    //physics:
    struct Body {
        glm::vec2 pos = glm::vec2(0.0f);
        glm::vec2 vel = glm::vec2(0.0f);
        float angle = 0.0f;
        float omega = 0.0f;
        float radius = 1.0f;
        float inv_mass = 1.0f;
    };

    enum : uint32_t { BodyCount = 20 };

    struct State {
        std::array< Body, BodyCount > bodies;
        float gravity = 0.0f;
        uint32_t time_left = 0;
        uint32_t launched = 0;
        uint32_t spawn_count = 0;
        uint32_t tick = 0;
    };

    struct Input {
        uint32_t flip = 0;
        uint32_t launch = 0;
        glm::vec2 launch_vel = glm::vec2(0.0f);
    };

    State state;

    void reset();
    void step(Input const &input);
	static uint32_t hash_state(State const &to_hash);

	std::vector< State > history;
	bool rewind_held = false;

	//replay:
	std::vector< Input > inputs;
	std::vector< Input > replay;
	bool replaying = false;
	uint32_t replay_expected_hash = 0;
	std::string replay_message = "";

	void start_replay(std::vector< Input > const &replay_inputs, uint32_t expected_hash);

    //player controls
    Input pending;
    bool paused = false;
    float accumulator = 0.0f;

    glm::vec2 mouse_world = glm::vec2(0.0f);
    glm::vec2 camera_center() const;
    glm::vec2 launch_velocity() const;

    float best_distance = 0.0f;
};