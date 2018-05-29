from PyQt5.QtWidgets import QLabel, QLineEdit, QDialog

from .ui_ScanReviewDialog import Ui_dlgScanView
from .widgets.ScanViewWidget import ScanViewWidget

import numpy as np

class ScanReviewDialog(QDialog):
    def __init__(self, scan):
        super().__init__()
        self.ui = Ui_dlgScanView()
        self.ui.setupUi(self)

        self._scan = scan

        self._accepted = False
        self._repeat = False

        # need to initialize text objects here so references stay in memory
        self._metadata_controls = {}
        for field, info in self._scan.metadata.items():
            self.add_metadata_control(field)

        self.initialize()

    def add_metadata_control(self, field):
        txt = QLineEdit()
        txt.setText(self._scan.metadata[field]['value'])
        if self._scan.metadata[field]['mandatory']:
            txt.setEnabled(False)
        txt.textChanged.connect(lambda: self.on_metadata_text_changed(field))

        self._metadata_controls[field] = txt

    def initialize(self):
        # scan view widget
        svw = ScanViewWidget(self._scan)
        self.ui.verticalLayout_2.insertWidget(0, svw)

        # title editing
        self.ui.lblScanTitle.setText(self._scan.title)
        self.ui.txtEditTitle.setParent(None)
        self.ui.txtEditTitle.returnPressed.connect(self.on_edit_title_return_pressed)
        self.ui.btnEditTitle.clicked.connect(self.on_edit_title_click)

        self.ui.btnSave.clicked.connect(self.on_save_click)
        self.ui.btnRepeatScan.clicked.connect(self.on_repeat_click)
        self.ui.lblScanKind.setText('{} scan'.format(self._scan.kind))
        self.ui.lblScanFile.setText('{} scan'.format(self._scan.file))

        #w = self.ui.lblScanImage.width()
        #h = self.ui.lblScanImage.height()
        #self.ui.lblScanImage.setPixmap(self._scan.make_histogram(int(w), int(h)))

        # fill in labels
        chr = 'V' if self._scan.kind == 'Vertical' else 'H'
        self.ui.lblScanStepperPoints.setText('[{}, {}], step={}'.format(
            self._scan.settings['txt{}MinPos'.format(chr)],
            self._scan.settings['txt{}MaxPos'.format(chr)],
            self._scan.settings['txt{}StepPos'.format(chr)],
        ))
        self.ui.lblScanVregPoints.setText('[{}, {}], step={}'.format(
            self._scan.settings['txt{}MinV'.format(chr)],
            self._scan.settings['txt{}MaxV'.format(chr)],
            self._scan.settings['txt{}StepV'.format(chr)],
        ))

        if not self._scan.settings['chkSaveImage']:
            self.ui.lblScanImageSaved.hide()

        lbl = QLabel('Time:')
        txt = QLineEdit()
        txt.setText(self._scan.time_string('%Y/%m/%d %H:%M %p'))
        txt.setEnabled(False)
        self.ui.layoutMetadata.addWidget(lbl, 0, 0)
        self.ui.layoutMetadata.addWidget(txt, 0, 2)

        sorted_fields = sorted([(name, info) for name, info in self._scan.metadata.items()],
            key=lambda x: x[1]['order'])

        for field, info in sorted_fields:
            lbl = QLabel(field + ':')
            txt = self._metadata_controls[field]

            self.ui.layoutMetadata.addWidget(lbl, info['order'] + 1, 0)
            self.ui.layoutMetadata.addWidget(txt, info['order'] + 1, 2)

    def on_metadata_text_changed(self, field):
        self._scan.metadata[field]['value'] = self._metadata_controls[field].text()

    def on_edit_title_click(self):
        self.ui.btnEditTitle.setParent(None)
        self.ui.layoutTitle.addWidget(self.ui.txtEditTitle)

    def on_edit_title_return_pressed(self):
        new_title = self.ui.txtEditTitle.text().strip()
        if new_title != '':
            self._scan.title = new_title
            self.ui.lblScanTitle.setText(new_title)

        self.ui.txtEditTitle.setParent(None)
        self.ui.layoutTitle.addWidget(self.ui.btnEditTitle)

    def on_save_click(self):
        # overwrite the scan file with new metadata
        with open(self._scan.file, 'w') as f:
            f.write(self._scan.preamble())
            for row in self._scan.data:
                f.write(','.join([str(_) for _ in row[0]]))
                f.write('\n')

        self._accepted = True
        self.accept()

    def on_repeat_click(self):
        self._repeat = True
        self.accept()

    def exec_(self):
        super(ScanReviewDialog, self).exec_()
        return self._accepted, self._repeat
