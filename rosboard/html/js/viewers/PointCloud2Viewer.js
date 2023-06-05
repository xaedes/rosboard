"use strict";

class PointCloud2Viewer extends Space3DViewer {
  _base64decode(base64) {
    var binary_string = window.atob(base64);
    var len = binary_string.length;
    var bytes = new Uint8Array(len);
    for (var i = 0; i < len; i++) {
        bytes[i] = binary_string.charCodeAt(i);
    }
    return bytes.buffer;
  }

  _getDataGetter(datatype, view) {
    // given a JS DataView view and a ROS PointField datatype,
    // fetch the getXXX function for that datatype.
    // (needed to bind it back to the view before returning, or else it doesn't work)
    // used for decoding raw point clouds

    switch(datatype) {
      case PointCloud2Viewer.INT8:
        return view.getInt8.bind(view);
      case PointCloud2Viewer.UINT8:
        return view.getUInt8.bind(view);
      case PointCloud2Viewer.INT16:
        return view.getInt16.bind(view);
      case PointCloud2Viewer.UINT16:
        return view.getUInt16.bind(view);
      case PointCloud2Viewer.INT32:
        return view.getInt32.bind(view);
      case PointCloud2Viewer.UINT32:
        return view.getUInt32.bind(view);
      case PointCloud2Viewer.FLOAT32:
        return view.getFloat32.bind(view);
      case PointCloud2Viewer.FLOAT64:
        return view.getFloat64.bind(view);
      default:
        return (offset, littleEndian) => {return 0.0};
    }
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    let points = null;

    if(msg.__comp) {
      points = this.decodeAndRenderCompressed(msg);
    } else {
      points = this.decodeAndRenderUncompressed(msg);
    }

  }
  
  decodeAndRenderCompressed(msg) {
    // decodes a uint16 lossy-compressed point cloud
    // basic explanation of algorithm:
    // - keep only x,y,z fields and throw away the rest
    // - throw away rows containing nans
    // - compress each number into a uint16 from 0 to 65535
    //   where 0 is the minimum value over the whole set and 65535 is the max value over the set
    //   so for example if the x values range from -95 m to 85 m, we encode x into a uint16 where
    //   0 represents -95 and 65535 represents 85
    // - provide the actual bounds (i.e. [-95, 85]) in a separate bounds field so it can be
    //   scaled back correctly by the decompressor (this function)
  
    let bounds = msg._data_uint16.bounds;
    let points_data = this._base64decode(msg._data_uint16.points);
    let points_view = new DataView(points_data);

    let points = new Float32Array(Math.round(points_data.byteLength / 2));

    let xrange = bounds[1] - bounds[0];
    let xmin = bounds[0];
    let yrange = bounds[3] - bounds[2];
    let ymin = bounds[2];
    let zrange = bounds[5] - bounds[4];
    let zmin = bounds[4];

    for(let i=0; i<points_data.byteLength/6; i++) {
      let offset = i * 6;
      points[3*i] = (points_view.getUint16(offset, true) / 65535) * xrange + xmin;
      points[3*i+1] = (points_view.getUint16(offset+2, true) / 65535) * yrange + ymin;
      points[3*i+2] = (points_view.getUint16(offset+4, true) / 65535) * zrange + zmin;
    }

    let aux_bounds = msg._data_uint16.aux_bounds;
    let aux_fields = msg._data_uint16.aux_fields;
    let aux_data = null;
    let aux_view = null;
    let aux = [];
    if (msg._data_uint16.aux_data.length > 0) {
      aux_data = this._base64decode(msg._data_uint16.aux_data);
      aux_view = new DataView(aux_data);
      aux = new Float32Array(Math.round(aux_data.byteLength / 2));
    }

    let aux_len = 0;
    if (aux_fields.length > 0) {
      aux_len = aux.length / aux_fields.length;
    }
    let aux_range = [];
    let aux_min = [];
    let aux_max = [];
    for (let i=0; i<aux_fields.length; i++) {
      let _min = aux_bounds[2*i+0];
      let _max = aux_bounds[2*i+1];
      let range = _max - _min;
      aux_range.push(range);
      aux_min.push(_min);
      aux_max.push(_max);
    }
    let aux_byte_step = 2 * aux_fields.length;
    for(let i=0; i<aux_len; i++) {
      let offset = i * aux_byte_step;
      for(let k=0; k<aux_fields.length; k++) {
        aux[aux_fields.length*i + k] = (aux_view.getUint16(offset+2*k, true) / 65535) * aux_range[k] + aux_min[k];
      }
    }

    this.draw([
      {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
      {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
      // {type: "points", data: points, zmin: zmin, zmax: zmin + zrange},
      {type: "points", data: points, zmin: zmin, zmax: zmin + zrange, auxData: aux, auxMin: aux_min, auxMax: aux_max, auxStep: aux_fields.length, auxFields:aux_fields},
    ]);
  }

  decodeAndRenderUncompressed(msg) {
    // decode an uncompressed pointcloud.
    // expects "data" field to be base64 encoded raw bytes but otherwise follows ROS standards

    let fields = {};
    let actualRecordSize = 0;

    msg.fields.forEach((field) => {
      fields[field.name] = field;
      if(!field.datatype in PointCloud2Viewer.SIZEOF) {
        this.error("Invalid PointCloud2 message: field " + field + " has invalid datatype = " + String(field.datatype));
        return;
      }
      actualRecordSize += PointCloud2Viewer.SIZEOF[field.datatype];
    });

    if(!("x" in fields) || !("y") in fields) {
      this.error("Cannot display PointCloud2 message: Must have at least 'x' and 'y' fields or I don't know how to display it.");
    }

    let data = this._base64decode(msg.data);

    if(!(msg.point_step * msg.width * msg.height === data.byteLength)) {
      this.error("Invalid PointCloud2: failed assertion: point_step * width * height === data.length");
      return;
    }

    let fieldNames = ["x", "y", "z"];
    
    let _auxFields = ["intensity", "reflectivity"];
    let auxFields = [];
    let auxStep = 0; 
    console.log("aux fields");
    
    for (let i=0; i<_auxFields.length; i++) {
      console.log("aux field");
      console.log(_auxFields[i]);
      if (_auxFields[i] in fields) {
        auxStep += 1;
        auxFields.push(_auxFields[i]);
      }
    }

    let auxData = [];
    if (auxStep > 0) {
      auxData = new Float32Array(Math.round(data.byteLength / msg.point_step * auxStep));
    } 
    let points = new Float32Array(Math.round(data.byteLength / msg.point_step * 3));
    let view = new DataView(data);
    let littleEndian = !msg.is_bigendian;

    // cache these into variables to avoid hitting hash repeatedly
    let xOffset = fields["x"].offset;
    let xDataGetter = this._getDataGetter(fields["x"].datatype, view);
    let yOffset = fields["y"].offset;
    let yDataGetter = this._getDataGetter(fields["y"].datatype, view);
    let zOffset = -1;
    let zDataGetter = null;
    if("z" in fields) {
      zOffset = fields["z"].offset;
      zDataGetter = this._getDataGetter(fields["z"].datatype, view);
    }

    let auxOffsets = [];
    let auxDataGetters = [];

    for (let i=0; i<auxFields.length; i++) {
      let field = auxFields[i];
      let _Offset = fields[field].offset;
      let _DataGetter = this._getDataGetter(fields[field].datatype, view);
      auxOffsets.push(_Offset);
      auxDataGetters.push(_DataGetter);
    }

    for(let i=0; i<data.byteLength/msg.point_step-1; i++) {
      let offset = i * msg.point_step;
      points[3*i] = xDataGetter(offset + xOffset, littleEndian); // x
      points[3*i+1] = yDataGetter(offset + yOffset, littleEndian); // y
      points[3*i+2] = zDataGetter(offset + zOffset, littleEndian); // y
      for (let k=0; k<auxDataGetters.length; k++) {
        auxData[auxStep*i+k] = auxDataGetters[k](offset + auxOffsets[k], littleEndian);
      }
    }

    this.draw([
      {type: "path", data: [0, 0, 0, 1], color: "#00f060", lineWidth: 2},
      {type: "path", data: [0, 0, 1, 0], color: "#f06060", lineWidth: 2},
      {type: "points", data: points, zmin: -2.0, zmax: 2.0, auxData: auxData, auxStep: auxStep, auxFields:auxFields},
    ]);
  }
}

// constants used for decoding raw point clouds (not used for decoding compressed)

PointCloud2Viewer.INT8 = 1;
PointCloud2Viewer.UINT8 = 2;
PointCloud2Viewer.INT16 = 3;
PointCloud2Viewer.UINT16 = 4;
PointCloud2Viewer.INT32 = 5;
PointCloud2Viewer.UINT32 = 6;
PointCloud2Viewer.FLOAT32 = 7;
PointCloud2Viewer.FLOAT64 = 8;
PointCloud2Viewer.SIZEOF = {
  [PointCloud2Viewer.INT8]: 1,
  [PointCloud2Viewer.UINT8]: 1,
  [PointCloud2Viewer.INT16]: 2,
  [PointCloud2Viewer.UINT16]: 2,
  [PointCloud2Viewer.INT32]: 4,
  [PointCloud2Viewer.UINT32]: 4,
  [PointCloud2Viewer.FLOAT32]: 4,
  [PointCloud2Viewer.FLOAT64]: 8,
}

// advertise this viewer to the system

PointCloud2Viewer.friendlyName = "Point cloud (3D)";
PointCloud2Viewer.supportedTypes = [
    "sensor_msgs/msg/PointCloud2",
];
PointCloud2Viewer.maxUpdateRate = 30.0;
Viewer.registerViewer(PointCloud2Viewer);
