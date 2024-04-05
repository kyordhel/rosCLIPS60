# ## ##################################################################
# controlgui.py
#
# Graphical User Interface for control GUI in python
#
# @author: Mauricio Matamoros
#
# ## ##################################################################
'''
@author: kyordhel, original GUI by arcra
'''

import os
import re
import sys
import os.path
import tkinter as tk
import tkinter.filedialog
import tkinter.messagebox

import rospy
from std_msgs.msg import String as rosstr

class ControlGUI():

    def __init__(self):
        self.root = tk.Tk()
        self.cmdVar = tk.StringVar()
        self.fileVar = tk.StringVar()
        self.runTimesVar = tk.StringVar(value=0)
        self.logLevelVar = tk.StringVar(value=None)
        self.__initGUI()
        self._setWatchFactButtonColor('gray')
        self._setWatchFuncButtonColor('gray')
        self._setWatchGlobButtonColor('gray')
        self._setWatchRuleButtonColor('gray')
        self._setWatchAllButtonColor('gray')
        self.__publish = None
    #end def


    def __initGUI(self):
        self.root.title('CLIPS pyControl GUI')
        self.root.bind_all('<KeyPress-Return>', self._issueRun)

        self.watchFactsButton = tk.Button(self.root, width=20, text='Watch Facts', bg='green', activebackground='green', command=self._toggleFactsWatched)
        self.watchRulesButton = tk.Button(self.root, width=20, text='Watch Rules', bg='green', activebackground='green', command=self._toggleRulesWatched)
        self.watchFunctionsButton = tk.Button(self.root, width=20, text='Watch Functions', bg='red', activebackground='red', command=self._toggleFunctionsWatched)
        self.watchGlobalsButton = tk.Button(self.root, width=20, text='Watch Globals', bg='green', activebackground='green', command=self._toggleGlobalsWatched)

        self.watchAllButton = tk.Button(self.root, text='WATCH ALL', bg='red', activebackground='red', command=self._toggleALLWatched)

        self.logLevelLabel = tk.Label(self.root, text='Log level:')
        self.logLevelINFO = tk.Radiobutton(self.root, text='INFO', value='INFO', variable=self.logLevelVar, command=self._issueLogLevel)
        self.logLevelWARNING = tk.Radiobutton(self.root, text='WARNING', value='WARNING', variable=self.logLevelVar, command=self._issueLogLevel)
        self.logLevelERROR = tk.Radiobutton(self.root, text='ERROR', value='ERROR', variable=self.logLevelVar, command=self._issueLogLevel)

        self.cmdFrame = tk.Frame(self.root)

        self.cmdLabel = tk.Label(self.cmdFrame, text='Enter command:')
        self.cmdEntry = tk.Entry(self.cmdFrame, width=56, textvariable=self.cmdVar)
        self.cmdButton = tk.Button(self.root, width=20, text='SEND COMMAND', bg='blue', activebackground='blue', fg='white', activeforeground='white', command=self._issueCommand)

        self.loadFrame   = tk.Frame(self.root)

        self.fileLabel = tk.Label(self.loadFrame, text='File:')
        self.fileEntry = tk.Entry(self.loadFrame, width=66, textvariable=self.fileVar)
        self.loadButton = tk.Button(self.root, width=20, text='LOAD FILE', bg='blue', activebackground='blue', fg='white', activeforeground='white', command=self._issueLoadFile)

        self.printFactsButton = tk.Button(self.root, width=20, text='Print Facts', bg='white', activebackground='white', command=self._issuePrintFacts)
        self.printRulesButton = tk.Button(self.root, width=20, text='Print Rules', bg='white', activebackground='white', command=self._issuePrintRules)
        self.printAgendaButton = tk.Button(self.root, width=20, text='Print Agenda', bg='white', activebackground='white', command=self._issuePrintAgenda)
        self.resetButton = tk.Button(self.root, width=20, text='RESET', bg='blue', activebackground='blue', fg='white', activeforeground='white', command=self._issueReset)

        self.timesFrame = tk.Frame(self.root)

        vcmd = (self.root.register(self._validateRunTimes), '%P')
        self.runTimesLabel = tk.Label(self.timesFrame, text='Run # times (0 to run ALL): ')
        self.runTimesEntry = tk.Entry(self.timesFrame, width=3, textvariable=self.runTimesVar, validate='all', validatecommand=vcmd)
        self.runButton = tk.Button(self.root, text='RUN', bg='blue', activebackground='blue', fg='white', activeforeground='white', command=self._issueRun)

        self.watchFunctionsButton.grid({'row':0, 'column': 0})
        self.watchGlobalsButton.grid({'row':0, 'column': 1})
        self.watchFactsButton.grid({'row':0, 'column': 2})
        self.watchRulesButton.grid({'row':0, 'column': 3})
        self.watchAllButton.grid({'row': 1, 'column': 0, 'columnspan': 4, 'sticky': tk.E+tk.W})

        self.logLevelLabel.grid({'row': 2, 'column': 0})
        self.logLevelINFO.grid({'row': 2, 'column': 1})
        self.logLevelWARNING.grid({'row': 2, 'column': 2})
        self.logLevelERROR.grid({'row': 2, 'column': 3})

        self.cmdFrame.grid({'row': 3, 'column': 0, 'columnspan': 3, 'sticky': tk.E+tk.W})
        self.cmdLabel.grid({'row': 0, 'column': 0})
        self.cmdEntry.grid({'row': 0, 'column': 1, 'sticky': tk.E+tk.W})
        self.cmdButton.grid({'row': 3, 'column': 3})

        self.loadFrame.grid({'row': 4, 'column': 0, 'columnspan': 3, 'sticky': tk.E+tk.W})
        self.fileLabel.grid({'row': 0, 'column': 0})
        self.fileEntry.grid({'row': 0, 'column': 1, 'sticky': tk.E+tk.W})
        self.loadButton.grid({'row': 4, 'column': 3})

        self.printFactsButton.grid({'row': 5, 'column': 0})
        self.printRulesButton.grid({'row': 5, 'column': 1})
        self.printAgendaButton.grid({'row': 5, 'column': 2})
        self.resetButton.grid({'row': 5, 'column': 3})

        self.timesFrame.grid({'row': 6, 'column': 0, 'columnspan': 3, 'sticky': tk.E})
        self.runTimesLabel.grid({'row': 0, 'column': 0})
        self.runTimesEntry.grid({'row': 0, 'column': 1})
        self.runButton.grid({'row': 6, 'column': 3, 'sticky': tk.N+tk.S+tk.E+tk.W})
    #end def


    # ## ##############################################################
    #
    # Properties
    #
    # ## ##############################################################
    @property
    def filePath(self):
        value = self.fileVar.get()
        if value is None or value == '' or not os.path.isfile(value):
            return
        return value
        # return self._fileName
    #end def

    @filePath.setter
    def filePath(self, value:str):
        if value is None or not os.path.isfile(value):
            self.fileVar.set('')
        else:
            self.fileVar.set(value)
        # self._fileName = value
    #end def


    @property
    def runTimes(self):
        try:
            return int(self.runTimesVar.get())
        except:
            return 0
    #end def

    @runTimes.setter
    def runTimes(self, value:int):
        self._runTimes = value
        self.runTimesVar.set(str(value))
    #end def


    # ## ##############################################################
    #
    # Public methods
    #
    # ## ##############################################################
    def loop(self):
        tk.mainloop()
        pass
    #end def


    def setPublisherFunc(self, f:callable):
        self.__publish = f
    #end def


    def setWatchFlags(self, flagStr:int):
        self._setWatchFactButtonColor('green' if flagStr & 0x80 else 'red')
        self._setWatchFuncButtonColor('green' if flagStr & 0x40 else 'red')
        self._setWatchGlobButtonColor('green' if flagStr & 0x01 else 'red')
        self._setWatchRuleButtonColor('green' if flagStr & 0x02 else 'red')
        self._setWatchAllButtonColor( 'green' if flagStr & 0xc3 else 'red')
    #end def


    # ## ##############################################################
    #
    # Private, GUI-related methods
    #
    # ## ##############################################################
    def _issueCommand(self):
        self._sendCommand(f'raw { self.cmdVar.get() }')
    #end def


    def _issueLoadFile(self):
        if self.filePath is None:
            self._showPickFileDialog()
            if self.filePath is None:
                return

        self._sendCommand(f'load { self.filePath }')
        # print('File Loaded!')
    #end def


    def _issueLogLevel(self):
        lvl = {
            'ERROR'   : 1,
            'WARNING' : 2,
            'INFO'    : 3,
        }.get(self.logLevelVar.get(), None);
        if lvl is None:
            return
        self._sendCommand(f'(bind ?*logLevel* "{ lvl }")')
    #end def


    def _issuePrintAgenda(self):
        self._sendCommand('print agenda')
    #end def


    def _issuePrintFacts(self):
        self._sendCommand('print facts')
    #end def


    def _issuePrintRules(self):
        self._sendCommand('print rules')
    #end def


    def _issueReset(self):
        self._sendCommand('reset')
        pass
    #end def


    def _issueRun(self):
        self._sendCommand(f'run { self.runTimes if self.runTimes > 0 else -1 }')
        pass
    #end def


    def _setWatchFactButtonColor(self, color):
        # 'red' | 'green'
        self.watchFactsButton['bg'] = color
        self.watchFactsButton['activebackground'] = color
    #end def


    def _setWatchFuncButtonColor(self, color):
        self.watchFunctionsButton['bg'] = color
        self.watchFunctionsButton['activebackground'] = color
    #end def


    def _setWatchGlobButtonColor(self, color):
        self.watchGlobalsButton['bg'] = color
        self.watchGlobalsButton['activebackground'] = color
    #end def


    def _setWatchRuleButtonColor(self, color):
        self.watchRulesButton['bg'] = color
        self.watchRulesButton['activebackground'] = color
    #end def


    def _setWatchAllButtonColor(self, color):
        self.watchAllButton['bg'] = color
        self.watchAllButton['activebackground'] = color
    #end def


    def _showPickFileDialog(self):
        ft = [
            ('All possible files', '.clp'),
            ('All possible files', '.dat'),
            ('All possible files', '.lst'),
            ('CLIPS Batch file', '.clp'),
            ('File list', '.dat'),
            ('File list', '.lst')
        ]
        self.filePath = tk.filedialog.askopenfilename(filetypes=ft)
        # self.filePath = tk.filedialog.Open(filetypes=ft)
    #end def


    def _toggleFactsWatched(self):
        self._sendCommand('watch facts')
    #end def


    def _toggleRulesWatched(self):
        self._sendCommand('watch rules')
    #end def


    def _toggleFunctionsWatched(self):
        self._sendCommand('watch functions')
    #end def


    def _toggleGlobalsWatched(self):
        self._sendCommand('watch globals')
    #end def


    def _toggleALLWatched(self):
        self._sendCommand('watch facts')
        self._sendCommand('watch rules')
        self._sendCommand('watch functions')
        self._sendCommand('watch globals')
    #end def


    def _validateRunTimes(self, value):
        return str.isdigit(value) or value == ''
    #end def


    # ## ##############################################################
    #
    # Private, ROS-related methods
    #
    # ## ##############################################################
    def _sendCommand(self, cmd):
        if not self.__publish:
            return
        self.__publish(f'\0{cmd}')
    #end def

#end class
