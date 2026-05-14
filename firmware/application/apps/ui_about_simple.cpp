#include "ui_about_simple.hpp"
#include <string_view>

#define ROLL_SPEED_FRAME_PER_LINE 60
// cuz frame rate of pp screen is probably 60, scroll per sec

namespace ui {

// Information: a line starting with a '#' will be yellow coloured
constexpr std::string_view mayhem_information_list[] = {
    "#****** Porta-433 ******",
    " ",
    "  https://discord.hackrf.app",
    " ",
    "#**** Contributors ****",
    " ",
    "  - Mayhem community",
    "  - RTL433 community",
    "  - HackRf community",
    "  - Forged by",
    "  - HTotoo",
    " "};

AboutView::AboutView(NavigationView& nav) {
    add_children({&menu_view,
                  &button_ok});

    button_ok.on_select = [&nav](Button&) {
        nav.pop();
    };

    menu_view.on_left = [this]() {
        button_ok.focus();
    };

    menu_view.on_right = [this]() {
        button_ok.focus();
    };

    for (auto& authors_line : mayhem_information_list) {
        // if it's starting with #, it's a title and we have to substract the '#' and paint yellow
        if (authors_line.size() > 0) {
            if (authors_line[0] == '#') {
                menu_view.add_item(
                    {(std::string)authors_line.substr(1, authors_line.size() - 1),
                     ui::Theme::getInstance()->fg_yellow->foreground,
                     nullptr,
                     nullptr});
            } else {
                menu_view.add_item(
                    {(std::string)authors_line,
                     Theme::getInstance()->bg_darkest->foreground,
                     nullptr,
                     nullptr});
            }
        }
    }
}

void AboutView::on_frame_sync() {
    if (interacted) return;

    if (frame_sync_count++ % ROLL_SPEED_FRAME_PER_LINE == 0) {
        const auto current = menu_view.highlighted_index();
        const auto count = menu_view.item_count();

        if (current < count - 1) {
            menu_view.set_highlighted(current + 1);
        } else {
            menu_view.set_highlighted(0);
        }
    }
}

void AboutView::focus() {
    button_ok.focus();
    menu_view.set_highlighted(3);  // contributors block starting line
}

bool AboutView::on_touch(const TouchEvent) {
    interacted = true;
    return false;
}

bool AboutView::on_key(const KeyEvent) {
    interacted = true;
    return false;
}

bool AboutView::on_encoder(const EncoderEvent) {
    interacted = true;
    menu_view.focus();
    return false;
}

} /* namespace ui */
