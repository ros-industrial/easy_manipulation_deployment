// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gui/replacewarning.h"
#include <string>

#include "gui/ui_replacewarning.h"

ReplaceWarning::ReplaceWarning(QWidget * parent)
: QDialog(parent),
  ui(new Ui::ReplaceWarning)
{
  ui->setupUi(this);
}

ReplaceWarning::~ReplaceWarning()
{
  delete ui;
}

void ReplaceWarning::set_label(std::string input)
{
  ui->warning_label->setText(QString::fromStdString(input));
}
void ReplaceWarning::on_ok_clicked()
{
  decision = true;
  this->close();
}

void ReplaceWarning::on_cancel_clicked()
{
  decision = false;
  this->close();
}
