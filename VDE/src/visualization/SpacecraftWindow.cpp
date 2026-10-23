#include "ui/SpacecraftWindow.hpp"

#include "ui/DockLayout.hpp"
#include "ui/Toolbar.hpp"
#include "ui/Widgets.hpp"
#include "ui/WindowNames.hpp"
#include "vde/AppState.hpp"
#include "visualization/SpacecraftScene.hpp"

#include <imgui.h>

namespace vde::ui {

SpacecraftWindow::SpacecraftWindow(AppState& app)
	: m_app(app), m_scene(std::make_unique<render::SpacecraftScene>()) {}

void SpacecraftWindow::draw(const std::function<void()>& replayBar) {
	m_graphs.sample(m_app.parameters(), m_app.spacecraftSnapshot());

	const bool visible = ImGui::Begin(names::kSpacecraftWindow);
	m_focused = visible && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows | ImGuiFocusedFlags_DockHierarchy);
	if (visible) {
		ToolbarOptions opt;
		opt.view         = ViewId::Spacecraft;
		opt.recordTag    = "spacecraft_view";
		opt.exportPrefix = "spacecraft_run";
		opt.replayMode   = m_app.replayOnSpacecraft();
		opt.timeS        = m_app.spacecraftSnapshot().timeS;
		drawToolbar(m_app, opt);
	}
	DockLayout::submitInner(DockLayout::spacecraftDockspaceId(), visible);
	ImGui::End();

	drawPanel();
	drawView(replayBar);
	drawGraphs();
}

void SpacecraftWindow::drawPanel() {
	const SpacecraftBundle& sc = m_app.spacecraftSnapshot().spacecraft;
	if (ImGui::Begin(names::kScPanel)) {
		if (m_panel.popped) poppedOutPlaceholder(m_panel.popped);
		else drawSpacecraftValuesPanel(sc, m_panel);
	}
	ImGui::End();
	poppedOutWindow("Control Panel – Spacecraft values###ScPanelPopped", m_panel.popped, ImVec2(420, 860),
					[&] { drawSpacecraftValuesPanel(sc, m_panel); });
}

void SpacecraftWindow::drawView(const std::function<void()>& replayBar) {
	const SimSnapshot& snap = m_app.spacecraftSnapshot();
	IRecorder* rec = &m_app.recorder(ViewId::Spacecraft);
	auto body = [&] { m_scene.draw(snap, m_viewPopped, rec, replayBar); };

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
	const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
	if (ImGui::Begin(names::kScView, nullptr, flags)) {
		if (m_viewPopped) poppedOutPlaceholder(m_viewPopped);
		else body();
	}
	ImGui::End();
	poppedOutWindow("Spacecraft View###ScViewPopped", m_viewPopped, ImVec2(1100, 800), body, flags);
	ImGui::PopStyleVar();
}

void SpacecraftWindow::drawGraphs() {
	if (ImGui::Begin(names::kScGraphs)) m_graphs.draw(m_app.parameters());
	ImGui::End();
	m_graphs.drawPopOuts();
}

} // namespace vde::ui
