#include "test_common.h"

#include "../BaseElement.h"

void test_widgets() {
    using Ren::Vec2f;
    using Ren::Vec2i;

    {
        // BaseElement tests
        {
            // Simple element
            Gui::RootElement root({ 1000, 1000 });
            Gui::BaseElement el({ -0.5f, -0.5f }, { 1, 1 }, &root);

            require(el.pos() == Vec2f(-0.5f, -0.5f));
            require(el.size() == Vec2f(1, 1));
            require(el.pos_px() == Vec2i(250, 250));
            require(el.size_px() == Vec2i(500, 500));

            root.set_zone({ 2000, 2000 });
            el.Resize(&root);
            el.Resize(&root);

            require(el.pos() == Vec2f(-0.5f, -0.5f));
            require(el.size() == Vec2f(1, 1));
            require(el.pos_px() == Vec2i(500, 500));
            require(el.size_px() == Vec2i(1000, 1000));

            root.set_zone({ 1000, 1000 });
            el.Resize({ 0, 0 }, { 0.5f, 0.5f }, &root);

            require(el.pos() == Vec2f(0, 0));
            require(el.size() == Vec2f(0.5f, 0.5f));
            require(el.pos_px() == Vec2i(500, 500));
            require(el.size_px() == Vec2i(250, 250));

            require(el.Check(Vec2f(0.25f, 0.25f)));
            require(!el.Check(Vec2f(-0.25f, 0.25f)));
            require(!el.Check(Vec2f(-0.25f, -0.25f)));
            require(!el.Check(Vec2f(0.25f, -0.25f)));

            require(el.Check(Vec2i(600, 600)));
            require(!el.Check(Vec2i(-600, 600)));
            require(!el.Check(Vec2i(-600, -600)));
            require(!el.Check(Vec2i(600, -600)));
        }

        {
            // Parenting
            Gui::RootElement root({ 1000, 1000 });
            Gui::BaseElement par_el({ 0, 0 }, { 1, 1 }, &root);
            Gui::BaseElement child_el({ 0, 0 }, { 1, 1 }, &par_el);

            require(child_el.pos() == Vec2f(0.5f, 0.5f));
            require(child_el.size() == Vec2f(0.5f, 0.5f));
            require(child_el.pos_px() == Vec2i(750, 750));
            require(child_el.size_px() == Vec2i(250, 250));

            par_el.Resize(&root);
            child_el.Resize(&par_el);

            require(child_el.pos() == Vec2f(0.5f, 0.5f));
            require(child_el.size() == Vec2f(0.5f, 0.5f));
            require(child_el.pos_px() == Vec2i(750, 750));
            require(child_el.size_px() == Vec2i(250, 250));

            par_el.Resize({ 0.5f, 0.5f }, { 0.5f, 0.5f }, &root);
            child_el.Resize(&par_el);

            require(child_el.pos() == Vec2f(0.75f, 0.75f));
            require(child_el.size() == Vec2f(0.25f, 0.25f));
            require(child_el.pos_px() == Vec2i(875, 875));
            require(child_el.size_px() == Vec2i(125, 125));
        }
    }

    {
        // LinearLayout tests
        // TODO
    }

}
