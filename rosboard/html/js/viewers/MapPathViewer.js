"use strict";

class MapPathViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewer = $('<div></div>')
      .css({'font-size': '11pt'
    , "filter": "invert(100%) saturate(50%)"})
      .appendTo(this.card.content);

    this.mapId = "map-" + Math.floor(Math.random()*10000);

    this.map = $('<div id="' + this.mapId + '"></div>')
      .css({
        "height": "250px",
      })
      .appendTo(this.viewer);

    this.mapLeaflet = L.map(this.mapId, {
      minZoom: 0,
      maxZoom: 30,
      zoomDelta: 0.25,
      zoomSnap: 0
    }).setView([51.505,-0.09], 15);

    // this.mapLeaflet.dragging.disable();

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(this.mapLeaflet);

    this.polyline = null;
  }

  onData(msg) {
      this.card.title.text(msg._topic_name);
      if(this.polyline) this.mapLeaflet.removeLayer(this.polyline);

      let points = [];
      for(let i = 0; i < msg.points.length; i++) {
        points.push(L.latLng(
          msg.points[i].latitude,
          msg.points[i].longitude
        ));
      }
      this.polyline = L.polyline(points, {
        weight: 2,
        smoothFactor: 0.0
      });
      this.polyline.addTo(this.mapLeaflet);
      if (points.length > 0) {
        let bounds = L.latLngBounds(points);
        this.mapLeaflet.fitBounds(bounds, {
          padding: [10, 10]
        });
      }
  }
}

MapPathViewer.friendlyName = "Street Map";

MapPathViewer.supportedTypes = [
    "rosboard/msg/NavSatFixPath",
];

MapPathViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(MapPathViewer);