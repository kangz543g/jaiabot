import { Map } from "ol"
import { Coordinate } from "ol/coordinate"
import { toLonLat, fromLonLat } from "ol/proj"
import { GeographicCoordinate } from "./JAIAProtobuf"

let abs = Math.abs

export function formatLatitude(lat: number, prec=5) {
    if (lat == null) {
        return "?"
    } 
    if (lat > 0) {
        return abs(lat).toFixed(prec) + "° N"
    }
    else {
        return abs(lat).toFixed(prec) + "° S"
    }
}

export function formatLongitude(lon: number, prec=5) {
    if (lon == null) {
        return "?"
    } 
    if (lon > 0) {
        return abs(lon).toFixed(prec) + "° E"
    }
    else {
        return abs(lon).toFixed(prec) + "° W"
    }
}

export function formatAttitudeAngle(angleDegrees: number, prec=2) {
    if (angleDegrees == null) {
        return "?"
    }
    return angleDegrees.toFixed(prec) + '°'
}

export function deepcopy<T>(aObject: T): T {
    // Prevent undefined objects
    // if (!aObject) return aObject;
  
    let bObject: any = Array.isArray(aObject) ? [] : {};
  
    let value;
    for (const key in aObject) {
  
      // Prevent self-references to parent object
      // if (Object.is(aObject[key], aObject)) continue;
      
      value = aObject[key];
  
      bObject[key] = (typeof value === "object") ? deepcopy(value) : value;
    }
  
    return bObject;
}

export function equalValues(a: any, b: any) {
    return JSON.stringify(a) == JSON.stringify(b)
}

export function randomBase57(stringLength: number) {
    const base75Chars = '123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstvwxyz'

    var s = ''
    for (let i = 0; i < stringLength; i++) {
        s = s.concat(base75Chars[Math.floor(Math.random() * base75Chars.length)])
    }
    return s
}

export function downloadToFile(data: string, mimeType: string, fileName: string) {
    const blob = new Blob([data], {type: mimeType})

    var link = window.document.createElement('a')
    link.href = window.URL.createObjectURL(blob)
    // Construct filename dynamically and set to link.download
    link.download = fileName
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
}

// getGeographicCoordinate()
//   Returns the GeographicCoordinate of an OpenLayers coordinate on a map
//   
//   Inputs
//     coordinate: coordinate to convert
//     map: an OpenLayers map that the coordinates refer to
export function getGeographicCoordinate(coordinate: Coordinate, map: Map) {
    const lonLat = toLonLat(coordinate, map.getView().getProjection())
    const geographicCoordinate: GeographicCoordinate = {
        lon: lonLat[0],
        lat: lonLat[1]
    }

    return geographicCoordinate
}

// getMapCoordinate()
//   Returns the OpenLayers Coordinate of an GeographicCoordinate on a map
//   
//   Inputs
//     coordinate: coordinate to convert
//     map: an OpenLayers map that the coordinates refer to
export function getMapCoordinate(coordinate: GeographicCoordinate, map: Map) {
    if (coordinate == null) return null
    return fromLonLat([coordinate.lon, coordinate.lat], map.getView().getProjection())
}

/**
 * Gets the element with a certain id
 * 
 * @param id id of the element to get
 * @returns The element, if it exists
 */
export function getElementById<T>(id: string) {
    // In case they passed a jQuery id selector in
    id = id.replaceAll('#', '')
    return document.getElementById(id) as T
}