#pragma once
#include "ui/GraphGrid.hpp"
#include "ui/SceneView.hpp"
#include "ui/Toolbar.hpp"

#include <functional>

namespace vde { class AppState; }

namespace vde::ui {

// "VDE Docking Simulator – VDE": toolbar + inner dockspace with
// Control Panel – VDE values | VDE View / Graphs.
class VdeWindow {
public:
	explicit VdeWindow(AppState& app);
	// replayBar: drawn over the bottom of the view while replaying on it (may be empty).
	ToolbarAction draw(const std::function<void()>& replayBar);
	bool focused() const { return m_focused; }
	void requestFocus() { m_focusFrames = 2; }  // 2 frames: newly appearing windows grab focus too

private:
	void drawPanel();
	void drawView(const std::function<void()>& replayBar);
	void drawGraphs();

	AppState& m_app;
	SceneView m_scene;
	GraphGrid m_graphs{6, 3, "vde"};
	bool m_panelPopped = false;
	bool m_viewPopped  = false;
	bool m_focused     = false;
	int  m_focusFrames = 0;
};

} // namespace vde::ui
