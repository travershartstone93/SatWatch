"""Categorize BLE devices using appearance, service UUIDs, name, and heuristics."""

from oui_db import lookup_oui
from ble_company_db import lookup_company

# GAP Appearance categories (top 6 bits = category)
# https://www.bluetooth.com/specifications/assigned-numbers/
APPEARANCE_CATEGORIES = {
    0: "Unknown",
    1: "Phone",
    2: "Computer",
    3: "Watch",
    4: "Clock",
    5: "Display",
    6: "Remote",
    7: "Eyeglasses",
    8: "Tag",
    9: "Keychain",
    10: "Media Player",
    11: "Barcode Scanner",
    12: "Thermometer",
    13: "Heart Rate",
    14: "Blood Pressure",
    15: "HID",
    16: "Glucose",
    17: "Running/Walking",
    18: "Cycling",
    49: "Pulse Oximeter",
    50: "Weight Scale",
    51: "Personal Mobility",
    52: "Continuous Glucose",
    53: "Insulin Pump",
    54: "Medication Delivery",
    81: "Outdoor Sports",
}

# Known 16-bit service UUIDs -> category hints
SERVICE_UUID_HINTS = {
    0x1800: "Generic Access",
    0x1801: "Generic Attribute",
    0x1802: "Immediate Alert",
    0x1803: "Link Loss",
    0x1804: "Tx Power",
    0x180A: "Device Info",
    0x180D: "Heart Rate",
    0x180F: "Battery",
    0x1810: "Blood Pressure",
    0x1812: "HID",
    0x1816: "Cycling Speed",
    0x1818: "Cycling Power",
    0x1819: "Location/Navigation",
    0x181A: "Environmental Sensing",
    0x181C: "User Data",
    0x181D: "Weight Scale",
    0xFE9F: "Google Nearby",
    0xFD6F: "COVID Exposure",
    0xFE2C: "Google",
    0xFEAA: "Eddystone",
    0xFFF0: "Custom/Proprietary",
}

# Service UUID -> device category mapping
SERVICE_TO_CATEGORY = {
    0x180D: "Wearable",       # Heart Rate
    0x1812: "Input Device",   # HID
    0x1816: "Fitness",        # Cycling Speed
    0x1818: "Fitness",        # Cycling Power
    0x1819: "Navigation",     # Location
    0x181A: "Sensor",         # Environmental
    0x181D: "Health",         # Weight Scale
    0xFE9F: "Phone",          # Google Nearby (usually phones)
    0xFD6F: "Phone",          # COVID Exposure (phones)
    0xFEAA: "Beacon",         # Eddystone
}

# Name pattern -> category
NAME_PATTERNS = [
    (["iphone", "galaxy", "pixel", "oneplus", "xiaomi", "huawei", "samsung"],  "Phone"),
    (["watch", "band", "fitbit", "garmin", "amazfit", "mi band"],              "Wearable"),
    (["airpod", "buds", "headphone", "earphone", "jbl", "bose", "sony wh",
      "sony wf", "beats"],                                                      "Audio"),
    (["tile", "airtag", "smarttag", "chipolo", "nut"],                          "Tracker"),
    (["beacon", "ibeacon", "estimote", "kontakt"],                              "Beacon"),
    (["tv", "roku", "chromecast", "firestick", "appletv"],                      "TV/Media"),
    (["keyboard", "mouse", "logitech", "trackpad"],                             "Input Device"),
    (["thermometer", "temp", "hygrometer", "sensor"],                           "Sensor"),
    (["scale", "weight"],                                                        "Health"),
    (["printer", "hp ", "epson", "canon", "brother"],                           "Printer"),
    (["lock", "august", "yale", "schlage"],                                      "Smart Lock"),
    (["light", "bulb", "hue", "lifx", "yeelight", "wiz"],                      "Lighting"),
    (["girbau", "washer", "dryer", "laundry", "midea", "aircond"],               "Appliance"),
    (["esp", "arduino", "nrf"],                                                  "Dev Board"),
    (["router", "gateway", "mesh", "netgear", "wifi ap"],                        "Network"),
]

# BLE SIG company IDs -> (manufacturer, default category)
# https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers/
MANUFACTURER_DB = {
    0x004C: ("Apple", "Phone"),
    0x00E0: ("Google", "Phone"),
    0x0075: ("Samsung", "Phone"),
    0x0006: ("Microsoft", "Computer"),
    0x000F: ("Broadcom", None),
    0x000D: ("Texas Instruments", "Sensor"),
    0x0059: ("Nordic Semi", "IoT"),
    0x02E5: ("Espressif", "IoT"),
    0x0131: ("Huawei", "Phone"),
    0x038F: ("Xiaomi", "Phone"),
    0x0157: ("Anhui Huami (Amazfit)", "Wearable"),
    0x0087: ("Garmin", "Wearable"),
    0x01DA: ("Fitbit", "Wearable"),
    0x00B0: ("Qualcomm", None),
    0x0310: ("Tile", "Tracker"),
    0x0822: ("Chipolo", "Tracker"),
    0x0171: ("Amazon", "Smart Home"),
    0x030F: ("Bose", "Audio"),
    0x012D: ("Sony", "Audio"),
    0x000A: ("Qualcomm (CSR)", "Audio"),
    0x0154: ("Google (Nest)", "Smart Home"),
    0x0499: ("Ruuvi", "Sensor"),
    0x0672: ("Govee", "Lighting"),
    0x0001: ("Ericsson", None),
    0x0002: ("Nokia", "Phone"),
    0x0003: ("Intel", "Computer"),
    0x004F: ("Harman (JBL)", "Audio"),
    0x009E: ("Bose", "Audio"),
    0x0108: ("Sonos", "Audio"),
}


APPLE_DEVICE_TYPES = {
    1: "iPhone",
    2: "iPad",
    3: "Mac",
    4: "Apple Watch",
    6: "AirPods",
    9: "Apple TV",
    10: "HomePod",
    14: "HomePod mini",
}

APPLE_MESSAGE_TYPES = {
    0x02: "iBeacon",
    0x05: "AirDrop",
    0x06: "HomeKit",
    0x07: "Proximity Pairing",
    0x08: "Siri",
    0x09: "AirPlay",
    0x0C: "Handoff",
    0x0D: "Hotspot",
    0x0E: "WiFi Settings",
    0x0F: "Nearby",
    0x10: "Nearby Info",
    0x12: "FindMy",
}

APPLE_NEARBY_ACTIVITY = {
    0: "Idle",
    1: "Audio",
    2: "Phone Call",
    3: "Driving",
}

APPLE_DEV_TO_CATEGORY = {
    1: "Phone",        # iPhone
    2: "Phone",        # iPad
    3: "Computer",     # Mac
    4: "Wearable",     # Apple Watch
    6: "Audio",        # AirPods
    9: "TV/Media",     # Apple TV
    10: "Smart Home",  # HomePod
    14: "Smart Home",  # HomePod mini
}

APPLE_MSG_TO_CATEGORY = {
    0x02: "Beacon",    # iBeacon
    0x12: "Tracker",   # FindMy / AirTag
}


CATEGORY_ICONS = {
    "Phone": "\U0001f4f1",
    "Wearable": "⌚",
    "Audio": "\U0001f3a7",
    "Tracker": "\U0001f4cd",
    "Beacon": "\U0001f4e1",
    "TV/Media": "\U0001f4fa",
    "Input Device": "⌨",
    "Sensor": "\U0001f321",
    "Health": "⚕",
    "Printer": "\U0001f5a8",
    "Smart Lock": "\U0001f512",
    "Lighting": "\U0001f4a1",
    "Appliance": "\U0001f9f9",
    "Dev Board": "\U0001f4bb",
    "Network": "\U0001f310",
    "Computer": "\U0001f4bb",
    "IoT": "\U0001f4e1",
    "Smart Home": "\U0001f3e0",
    "Fitness": "\U0001f3c3",
    "Navigation": "\U0001f9ed",
    "Unknown": "❓",
}


def categorize_by_appearance(appearance: int) -> str | None:
    if not appearance:
        return None
    category_id = (appearance >> 6) & 0x3FF
    return APPEARANCE_CATEGORIES.get(category_id)


def categorize_by_service(svc_uuid: int) -> str | None:
    if not svc_uuid:
        return None
    return SERVICE_TO_CATEGORY.get(svc_uuid)


def categorize_by_name(name: str) -> str | None:
    if not name:
        return None
    lower = name.lower()
    for patterns, category in NAME_PATTERNS:
        for pat in patterns:
            if pat in lower:
                return category
    return None


def categorize_by_apple_type(apple_dev_type: int = 0, apple_msg_type: int = 0) -> str | None:
    cat = APPLE_MSG_TO_CATEGORY.get(apple_msg_type)
    if cat:
        return cat
    cat = APPLE_DEV_TO_CATEGORY.get(apple_dev_type)
    if cat:
        return cat
    return None


def get_apple_device_label(dev_type: int) -> str:
    return APPLE_DEVICE_TYPES.get(dev_type, "")


def get_apple_message_label(msg_type: int) -> str:
    return APPLE_MESSAGE_TYPES.get(msg_type, "")


def get_apple_activity_label(status_byte: int) -> str:
    return APPLE_NEARBY_ACTIVITY.get(status_byte & 0x0F, "")


def categorize_by_manufacturer(mfg_id: int) -> str | None:
    if not mfg_id:
        return None
    entry = MANUFACTURER_DB.get(mfg_id)
    if entry:
        return entry[1]  # default category (may be None)
    return None


def get_manufacturer_name(mfg_id: int) -> str:
    if not mfg_id:
        return ""
    entry = MANUFACTURER_DB.get(mfg_id)
    if entry:
        return entry[0]
    name = lookup_company(mfg_id)
    return name if name else f"0x{mfg_id:04X}"


def get_oui_manufacturer(mac: str) -> str:
    return lookup_oui(mac)


def extract_brand_from_name(name: str) -> str:
    """Extract manufacturer/brand from BLE device name."""
    if not name:
        return ""
    # Common patterns: "Brand_XXXX", "Brand-XXXX", "BrandModel"
    for sep in ["_", "-", " "]:
        if sep in name:
            brand = name.split(sep)[0]
            if len(brand) >= 3:
                return brand
    return ""


def get_best_manufacturer(mfg_id: int = 0, mac: str = "", name: str = "") -> str:
    """Get the best available manufacturer name from all sources."""
    if mfg_id:
        entry = MANUFACTURER_DB.get(mfg_id)
        if entry:
            return entry[0]
        company = lookup_company(mfg_id)
        if company:
            return company
    if mac and not (int(mac.split(":")[0], 16) & 0x02):
        oui = lookup_oui(mac)
        if oui:
            return oui
    brand = extract_brand_from_name(name)
    if brand:
        return brand
    return ""


DEVICE_TYPE_KEYWORDS = [
    (["air-cond", "air cond", "aircon", "midea", "daikin", "carrier", "trane",
      "gree", "hisense"], "Air Conditioner"),
    (["washer", "washing", "laundry", "girbau"], "Washing Machine"),
    (["dryer"], "Dryer"),
    (["dishwash"], "Dishwasher"),
    (["fridge", "refriger"], "Refrigerator"),
    (["oven", "microwave", "range"], "Oven"),
    (["vacuum", "roomba", "roborock"], "Robot Vacuum"),
    (["thermostat", "nest", "ecobee"], "Thermostat"),
    (["purifier", "humidif", "dehumid"], "Air Purifier"),
    (["router", "gateway", "mesh", "wifi ap"], "Router"),
    (["switch", "netgear", "cisco", "ubiquiti", "aruba"], "Network Switch"),
    (["printer", "hp deskjet", "hp officejet", "epson", "canon pixma",
      "brother"], "Printer"),
    (["tv", "roku", "firestick", "chromecast"], "Smart TV"),
    (["speaker", "sonos", "echo", "homepod", "alexa"], "Smart Speaker"),
    (["camera", "ring", "arlo", "wyze", "nest cam"], "Camera"),
    (["lock", "august", "yale", "schlage"], "Smart Lock"),
    (["light", "bulb", "hue", "lifx", "yeelight"], "Smart Light"),
    (["scale", "weight"], "Smart Scale"),
    (["keyboard"], "Keyboard"),
    (["mouse", "trackpad"], "Mouse"),
    (["headphone", "earphone"], "Headphones"),
    (["earbuds", "buds"], "Earbuds"),
    (["jbl", "bose", "sony wh", "beats solo", "beats studio",
      "sennheiser"], "Headphones"),
    (["airpod"], "AirPods"),
    (["watch", "band", "fitbit", "amazfit", "mi band"], "Fitness Tracker"),
    (["tile", "smarttag", "chipolo"], "Tracker Tag"),
    (["airtag"], "AirTag"),
    (["beacon", "ibeacon", "estimote", "kontakt"], "BLE Beacon"),
    (["sensor", "thermometer", "hygrometer"], "Sensor"),
    (["esp", "arduino", "nrf", "devkit"], "Dev Board"),
]

APPLE_DEVICE_TYPE_LABELS = {
    1: "iPhone",
    2: "iPad",
    3: "Mac",
    4: "Apple Watch",
    6: "AirPods",
    9: "Apple TV",
    10: "HomePod",
    14: "HomePod Mini",
}

APPLE_MSG_TYPE_LABELS = {
    0x02: "iBeacon",
    0x12: "AirTag / FindMy",
}


def get_device_type_label(name: str = "", manufacturer: str = "",
                           apple_dev_type: int = 0, apple_msg_type: int = 0,
                           category: str = "", mfg_id: int = 0) -> str:
    """Best-effort specific device type guess from all available signals."""
    if apple_msg_type in APPLE_MSG_TYPE_LABELS:
        return APPLE_MSG_TYPE_LABELS[apple_msg_type]
    if apple_dev_type in APPLE_DEVICE_TYPE_LABELS:
        return APPLE_DEVICE_TYPE_LABELS[apple_dev_type]
    search_str = f"{name} {manufacturer}".lower()
    for patterns, label in DEVICE_TYPE_KEYWORDS:
        for pat in patterns:
            if pat in search_str:
                return label
    if mfg_id == 0x004C:
        return "Apple Device"
    return category


def get_behavior_label(hits: int = 0, duration_s: float = 0,
                        rssi_mean: float = -100, dist_est_m: float = 33) -> str:
    """Generate a behavioral description for unknown devices."""
    parts = []
    if duration_s > 600:
        parts.append("Stationary")
    elif duration_s > 60:
        parts.append("Lingering")
    else:
        parts.append("Transient")
    if dist_est_m < 5:
        parts.append("Very Close")
    elif dist_est_m < 15:
        parts.append("Nearby")
    elif dist_est_m < 25:
        parts.append("Mid-range")
    else:
        parts.append("Distant")
    return ", ".join(parts)


COMPANY_NAME_HINTS = [
    (["apple"],         "Phone"),
    (["samsung"],       "Phone"),
    (["google"],        "Phone"),
    (["huawei"],        "Phone"),
    (["xiaomi", "redmi"], "Phone"),
    (["oneplus", "oppo", "vivo", "realme"], "Phone"),
    (["sony", "bose", "jbl", "harman", "beats", "sennheiser", "jabra"], "Audio"),
    (["fitbit", "garmin", "amazfit", "huami", "zepp"], "Wearable"),
    (["tile", "chipolo"], "Tracker"),
    (["espressif", "nordic", "silicon labs", "texas instrument", "microchip"], "IoT"),
    (["lg electron", "whirlpool", "electrolux", "bosch", "miele", "girbau",
      "midea", "haier", "daikin", "carrier", "trane", "honeywell", "air-cond",
      "hisense", "gree", "panasonic hvac"], "Appliance"),
    (["tp-link", "netgear", "cisco", "ubiquiti", "aruba", "ruckus", "juniper"], "Network"),
    (["philips", "signify", "ikea", "lutron", "leviton"], "Lighting"),
    (["amazon", "ring"], "Smart Home"),
    (["hp ", "hewlett", "epson", "canon", "brother", "lexmark"], "Printer"),
    (["intel", "dell", "lenovo", "microsoft", "asus"], "Computer"),
    (["roku", "sonos", "denon", "marantz"], "TV/Media"),
]


def categorize_by_company_name(name: str) -> str | None:
    """Guess category from OUI or BLE company name string."""
    if not name:
        return None
    lower = name.lower()
    for patterns, category in COMPANY_NAME_HINTS:
        for pat in patterns:
            if pat in lower:
                return category
    return None


def categorize_by_heuristics(connectable: bool, payload_len: int,
                              tx_power: int, adv_type: int,
                              le_mode: int = 0, is_random: bool = False) -> str | None:
    """Last-resort heuristic classification."""
    if not connectable and payload_len == 30:
        return "Beacon"
    if not connectable and payload_len == 31 and adv_type == 3:
        return "Beacon"
    if not connectable and tx_power != 127 and payload_len < 20:
        return "Beacon"
    if adv_type == 4:
        return None
    # Connectable + BR/EDR+LE + random MAC + typical phone payload size
    if connectable and le_mode == 2 and is_random and 25 <= payload_len <= 31:
        return "Phone"
    return None


def categorize_device(name: str = "", appearance: int = 0, svc_uuid: int = 0,
                       connectable: bool = True, payload_len: int = 0,
                       tx_power: int = 127, adv_type: int = 0,
                       mfg_id: int = 0, apple_dev_type: int = 0,
                       apple_msg_type: int = 0, mac: str = "",
                       le_mode: int = 0) -> str:
    """Classify a BLE device using all available signals, highest confidence first."""
    # 1. GAP Appearance (most reliable when present)
    cat = categorize_by_appearance(appearance)
    if cat and cat != "Unknown":
        return cat

    # 2. Service UUID
    cat = categorize_by_service(svc_uuid)
    if cat:
        return cat

    # 3. Name matching
    cat = categorize_by_name(name)
    if cat:
        return cat

    # 3.5. Apple Continuity device/message type
    cat = categorize_by_apple_type(apple_dev_type, apple_msg_type)
    if cat:
        return cat

    # 4. Manufacturer ID (curated DB)
    cat = categorize_by_manufacturer(mfg_id)
    if cat:
        return cat

    # 4.5. Full BLE company DB name → category guess
    if mfg_id:
        cat = categorize_by_company_name(lookup_company(mfg_id))
        if cat:
            return cat

    # 4.7. OUI (MAC prefix) → manufacturer → category guess
    if mac and not (int(mac.split(":")[0], 16) & 0x02):
        cat = categorize_by_company_name(lookup_oui(mac))
        if cat:
            return cat

    # 5. Payload/behavior heuristics
    is_random = bool(mac and (int(mac.split(":")[0], 16) & 0x02))
    cat = categorize_by_heuristics(connectable, payload_len, tx_power, adv_type,
                                    le_mode, is_random)
    if cat:
        return cat

    return "Unknown"


def get_service_label(svc_uuid: int) -> str:
    """Get human-readable service name."""
    if not svc_uuid:
        return ""
    return SERVICE_UUID_HINTS.get(svc_uuid, f"0x{svc_uuid:04X}")


def get_category_icon(category: str) -> str:
    return CATEGORY_ICONS.get(category, "")
