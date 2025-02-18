# Optimal Fuel Route Planner

A Django-based application that helps users find the most cost-effective route for fueling their vehicles across the USA. The application considers real-time fuel prices and optimizes the route to minimize total fuel costs.

## Features

- Route optimization between any two locations in the USA
- Real-time fuel price consideration
- Interactive map visualization with:
  - Optimized route display
  - Recommended gas stations for refueling
  - Distance and cost calculations
- Maximum range consideration (500 miles between stops)
- Dynamic programming optimization for minimal fuel costs

## Tech Stack

- **Backend**: Django
- **Data Processing**: Pandas, NumPy
- **Geospatial**: 
  - Geopy for geocoding
  - Folium for map visualization
  - OSRM for routing
  - Rtree for spatial indexing
- **APIs**:
  - Nominatim API for geocoding
  - OSRM API for routing

## Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd gas-assignment
```

2. Create and activate virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: .\venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Preprocess the dataset:
```bash
cd dataset_preprocessing
jupyter notebook
```
Run the notebooks in order:
- `csv_geocoding.ipynb`
- `get_exact_loc.ipynb`

5. Set up Django:
```bash
cd mapApp-1
python manage.py migrate
python manage.py runserver
```

## API Usage

### Route Optimization Endpoint

```http
GET /api/route/?start={start_address}&destination={end_address}
```

Example:
```http
GET /api/route/?start=New York, NY&destination=Los Angeles, CA
```

Response:
```json
{
    "route": {
        "geometry": {...},
        "distance": 2789.4,
        "duration": 144000
    },
    "gas_stations": [{
        "name": "Station Name",
        "latitude": 40.7128,
        "longitude": -74.0060,
        "price": 3.99,
        "distance_from_start": 250
    }],
    "min_money": 350.75
}
```

## Dataset

The project uses a fuel prices dataset containing:
- Gas station locations
- Current fuel prices
- Station details

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request