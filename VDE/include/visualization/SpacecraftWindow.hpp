#pragma once
#include "ui/GraphGrid.hpp"
#include "ui/SceneView.hpp"
#include "ui/SpacecraftValuesPanel.hpp"

#include <functional>

namespace vde { class AppState; }

namespace vde::ui {

// "VDE Docking Simulator – Spacecraft": toolbar (RECORD VIDEO, EXPORT DATA) +
// Control Panel – Spacecraft values | Spacecraft View / Graphs.
class SpacecraftWindow {
public:
	explicit SpacecraftWindow(AppState& app);
	// replayBar: drawn over the bottom of the view while replaying on it (may be empty).
	void draw(const std::function<void()>& replayBar);
	bool focused() const { return m_focused; }

private:
	void drawPanel();
	void drawView(const std::function<void()>& replayBar);
	void drawGraphs();

	AppState&            m_app;
	SceneView            m_scene;
	GraphGrid            m_graphs{6, 3, "spacecraft"};
	SpacecraftPanelState m_panel;
	bool                 m_viewPopped = false;
	bool                 m_focused = false;
};

} // namespace vde::ui
