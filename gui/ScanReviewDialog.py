from PyQt5.QtWidgets import QLabel, QLineEdit, QDialog

from .ui_ScanReviewDialog import Ui_dlgScanView

import numpy as np

class ScanReviewDialog(QDialog):
    def __init__(self, scan):
        super().__init__()
        self.ui = Ui_dlgScanView()
        self.ui.setupUi(self)

        self._scan = scan

        self._accepted = False

        self.initialize()

    def initialize(self):
        # title editing
        self.ui.txtEditTitle.setParent(None)
        self.ui.txtEditTitle.returnPressed.connect(self.on_edit_title_return_pressed)
        self.ui.btnEditTitle.clicked.connect(self.on_edit_title_click)

        self.ui.btnSave.clicked.connect(self.on_save_click)
        self.ui.lblScanKind.setText('{} scan'.format(self._scan.kind))
        self.ui.lblScanFile.setText('{} scan'.format(self._scan.file))

        w = self.ui.lblScanImage.width()
        h = self.ui.lblScanImage.height()
        self.ui.lblScanImage.setPixmap(self._scan.make_histogram(int(w), int(h)))

        # fill in labels
        stepper_point_count = len(np.unique(self._scan.data['pos']))
        stepper_min = min(self._scan.data['pos'])[0]
        stepper_max = max(self._scan.data['pos'])[0]
        stepper_step = 'None'
        if stepper_point_count > 1:
            stepper_step = np.unique(self._scan.data['pos'])[1] - np.unique(self._scan.data['pos'])[0]

        self.ui.lblScanStepperPoints.setText('[{}, {}], step={}'.format(stepper_min, stepper_max, stepper_step))

        vreg_point_count = len(np.unique(self._scan.data['v']))
        vreg_min = min(self._scan.data['v'])[0]
        vreg_max = max(self._scan.data['v'])[0]
        vreg_step = 'None'
        if vreg_point_count > 1:
            vreg_step = np.unique(self._scan.data['v'])[1] - np.unique(self._scan.data['v'])[0]

        self.ui.lblScanVregPoints.setText('[{}, {}], step={}'.format(vreg_min, vreg_max, vreg_step))

        sorted_fields = sorted([(name, info) for name, info in self._scan.metadata.items()],
            key=lambda x: x[1]['order'])

        if not self._scan.image_saved:
            self.ui.lblScanImageSaved.hide()

        lbl = QLabel('Time:')
        txt = QLineEdit()
        txt.setText(self._scan.time_string('%Y/%m/%d %H:%M %p'))
        txt.setEnabled(False)
        self.ui.layoutMetadata.addWidget(lbl, 0, 0)
        self.ui.layoutMetadata.addWidget(txt, 0, 2)


        for field, info in sorted_fields:
            lbl = QLabel(field + ':')
            txt = QLineEdit()
            txt.setText(info['value'])

            self.ui.layoutMetadata.addWidget(lbl, info['order'] + 1, 0)
            self.ui.layoutMetadata.addWidget(txt, info['order'] + 1, 2)

    def on_edit_title_click(self):
        self.ui.btnEditTitle.setParent(None)
        self.ui.layoutTitle.addWidget(self.ui.txtEditTitle)

    def on_edit_title_return_pressed(self):
        new_title = self.ui.txtEditTitle.text().strip()
        if new_title != '':
            self.ui.lblScanTitle.setText(new_title)

        self.ui.txtEditTitle.setParent(None)
        self.ui.layoutTitle.addWidget(self.ui.btnEditTitle)

    def on_save_click(self):
        self._accepted = True
        self.accept()

    def exec_(self):
        super(ScanReviewDialog, self).exec_()
        return self._accepted
