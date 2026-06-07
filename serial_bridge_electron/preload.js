'use strict';
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  getLogsDir:    ()      => ipcRenderer.invoke('get-logs-dir'),
  chooseLogsDir: ()      => ipcRenderer.invoke('choose-logs-dir'),
});
