#include "ui/MainUi.hpp"

#include "ui/WindowNames.hpp"
#include "vde/AppState.hpp"

#include <imgui.h>
#include <implot.h>

#include <cstdio>
#include <functional>

namespace vde::ui {

MainUi::MainUi(AppState& app) : m_app(app), m_vde(app), m_spacecraft(app), m_analysis(app) {}

void MainUi::frame() {
	drawMenuBar();
	m_layout.beginFrame();
	if (m_layout.builtThisFrame()) m_vde.requestFocus();  // VDE is the default tab

	// The playback bar is overlaid only on the view(s) chosen in the Replay dialog.
	const auto replayBar = [this] { drawReplayBar(m_app); };
	using BarFn = std::function<void()>;
	handle(m_vde.draw(m_app.replayOnVde() ? BarFn(replayBar) : BarFn()));
	m_spacecraft.draw(m_app.replayOnSpacecraft() ? BarFn(replayBar) : BarFn());
	m_analysis.draw();
	m_log.draw();
	m_configDialog.draw();
	m_replayDialog.draw(m_app);

	if (m_showDemo) {
		ImGui::ShowDemoWindow(&m_showDemo);
		ImPlot::ShowDemoWindow();
	}

	// OS window title follows whichever top-level window was focused last.
	if (m_vde.focused()) m_osTitle = names::kVdeWindow;
	if (m_spacecraft.focused()) m_osTitle = names::kSpacecraftWindow;
}

void MainUi::drawMenuBar() {
	if (!ImGui::BeginMainMenuBar()) return;
	if (ImGui::BeginMenu("View")) {
		if (ImGui::MenuItem("Reset layout")) m_layout.requestReset();
		ImGui::MenuItem("Log console", nullptr, &m_log.visible());
		ImGui::Separator();
		ImGui::MenuItem("ImGui / ImPlot demo", nullptr, &m_showDemo);
		ImGui::EndMenu();
	}
	// Diagnostics (coding standard §6): frame rate and time, fixed steps this frame and
	// their cost, steps dropped by the per-frame cap since start.
	const ImGuiIO& io = ImGui::GetIO();
	const FrameStats& fs = m_app.frameStats();
	char stats[160];
	std::snprintf(stats, sizeof(stats), "%.0f FPS · frame %.1f ms · %d step%s %.2f ms @ %.0f Hz · dropped %lld",
				  io.Framerate, io.DeltaTime * 1000.0f, fs.stepsThisFrame, fs.stepsThisFrame == 1 ? "" : "s",
				  fs.stepTimeMs, m_app.config().stepRateHz, fs.droppedStepsTotal);
	ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(stats).x - 16);
	ImGui::TextDisabled("%s", stats);
	ImGui::EndMainMenuBar();
}

void MainUi::handle(ToolbarAction action) {
	switch (action) {
	case ToolbarAction::OpenAnalysis:  m_analysis.open(); break;
	case ToolbarAction::Replay:        m_replayDialog.open(); break;
	case ToolbarAction::InitialConfig: m_configDialog.open(m_app.configLoader()); break;
	case ToolbarAction::None: break;
	}
}

} // namespace vde::ui
