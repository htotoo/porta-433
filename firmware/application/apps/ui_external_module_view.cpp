/*
 * Copyright (C) 2024 Bernd Herzog
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui_external_module_view.hpp"
#include "portapack.hpp"
#include "ui_standalone_view.hpp"

#include "i2cdevmanager.hpp"

#include <optional>

namespace ui {

void ExternalModuleView::focus() {
    dummy.focus();
}

void ExternalModuleView::on_tick_second() {
    i2cdev::I2CDevManager::manual_scan();

    text_header.set("No module connected");
    text_name.set("");
    text_version.set("");
    text_number_apps.set("");
    text_app1_name.set("");
    text_app2_name.set("");
    text_app3_name.set("");
    text_app4_name.set("");
    text_app5_name.set("");
    return;
}

}  // namespace ui
