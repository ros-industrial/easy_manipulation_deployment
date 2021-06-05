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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__REPLACEWARNING_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__REPLACEWARNING_H_

#include <QDialog>
#include <string>

namespace Ui
{
  class ReplaceWarning; // NOLINT
}

class ReplaceWarning: public QDialog
{
  Q_OBJECT

public:
  explicit ReplaceWarning(QWidget * parent = nullptr);
  void set_label(std::string input);
  bool decision = false;
  ~ReplaceWarning();

private slots:
  void on_ok_clicked();

  void on_cancel_clicked();

private:
  Ui::ReplaceWarning * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__REPLACEWARNING_H_
