#pragma once

#include <imgui/imgui.h>
#include <yoga/Yoga.h>

// Flex layout for the screens, computed by Yoga and read back as screen
// rectangles that ImGui widgets and draw-list calls are placed at. A web
// reference's flex, gap, padding and size values go on the nodes verbatim,
// through the Yoga calls named after the CSS properties; web defaults are on,
// so a node is a row that can shrink, as in a browser.
//
// The tree is built, laid out and thrown away each frame, like the ImGui
// widgets it places. Nothing here draws.

struct FlexRect
{
	ImVec2 min, max;
	float W() const { return max.x - min.x; }
	float H() const { return max.y - min.y; }
	ImVec2 Size() const { return ImVec2(W(), H()); }
	ImVec2 Center() const { return ImVec2((min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f); }
};

class FlexLayout
{
public:
	FlexLayout();
	~FlexLayout();
	FlexLayout(const FlexLayout &) = delete;
	FlexLayout &operator=(const FlexLayout &) = delete;

	YGNodeRef Root() const { return root; }

	// A new child appended to parent. The layout owns every node it hands out.
	YGNodeRef Add(YGNodeRef parent);
	YGNodeRef Row(YGNodeRef parent);
	// A leaf the size of one line of text in font; the caller draws the text.
	YGNodeRef Text(YGNodeRef parent, ImFont *font, const char *text);

	// Lay the tree out with the root's top-left corner at origin. A root with
	// no size of its own takes exactly width by height; pass YGUndefined for
	// a dimension the content should decide.
	void Compute(ImVec2 origin, float width, float height);

	// Screen rectangle of a node after Compute.
	FlexRect Rect(YGNodeRef node) const;

private:
	YGConfigRef config;
	YGNodeRef root;
	ImVec2 origin;
};
