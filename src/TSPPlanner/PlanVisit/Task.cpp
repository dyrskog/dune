//***************************************************************************
// Copyright 2007-2023 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: J Dyrskog                                                        *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace TSPPlanner
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author J Dyrskog
  namespace PlanVisit
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      std::vector<double> points; // Points of interest
      bool active; // Task activation status
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      // Points of interest
      std::vector<std::vector<double>> all_points;
      // Number of POIs
      int n_points;
      // Current vehicle position
      double current_pos[2];

      // TSP variables
      // Distance matrix
      std::vector<std::vector<double>> dist;
      // Min cost matrix
      std::vector<std::vector<double>> min_distances;
      // Min cost index matrix
      std::vector<std::vector<int>> path_indices;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        // Subscribe to estimated vehicle state
        bind<IMC::EstimatedState>(this);

        param("Points to Visit", m_args.points);

        param("Active", m_args.active)
        .defaultValue("true");
        paramActive(Tasks::Parameter::SCOPE_GLOBAL,
                    Tasks::Parameter::VISIBILITY_USER);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        n_points = (int) m_args.points.size()/2;
        // Allocate space for input points + initial position
        all_points.resize(n_points + 1, std::vector<double>(2));

        // Put current position + POIs in the same array
        all_points[0][0] = current_pos[0];
        all_points[0][1] = current_pos[1];
        spew("points: %f, %f", all_points[0][0], all_points[0][1]);
        for (int i = 0; i < n_points; i++){
          all_points[i+1][0] = Angles::radians(m_args.points[i*2]);
          all_points[i+1][1] = Angles::radians(m_args.points[(i*2)+1]);
          spew("points: %f, %f", all_points[i+1][0], all_points[i+1][1]);
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      //! Main loop.
      void
      onMain(void)
      {
        bool planGenerated = false;
        while (!stopping())
        {
          waitForMessages(1.0);

          if (current_pos[0] != 0 && !planGenerated){
            // Vehicle position is updated, generate plan
            generatePlan();
            planGenerated = true;
          }

        }
      }

      void
      consume(const IMC::EstimatedState* state)
      {
        if (!isActive())
            return;
        double m_lat;
        double m_lon;
        float m_height;
        Coordinates::toWGS84(*state, m_lat, m_lon, m_height);

        current_pos[0] = m_lat;
        current_pos[1] = m_lon;
      }

      //! On activation
      void
      onActivation(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      //! On deactivation
      void
      onDeactivation(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }

      void
      generatePlan(void)
      {
        n_points = all_points.size();
        // Allocate memory for matrices
        dist.resize(n_points + 1, std::vector<double>(n_points+1));
        min_distances.resize(n_points, std::vector<double>(1 << n_points));
        path_indices.resize(n_points, std::vector<int>(1 << n_points));
        
        // Create graph representation of distances
        // Vertices = points, edge weights = distances
        all_points[0][0] = current_pos[0]; // Update current position
        all_points[0][1] = current_pos[1];
        for (int i = 0; i < n_points; i++){
          for (int j = 0; j < n_points; j++){
            // Find distance from point i to every other point j
            double lat1 = all_points[i][0];
            double lon1 = all_points[i][1];
            double hae1 = 0;
            double lat2 = all_points[j][0];
            double lon2 = all_points[j][1];
            double hae2 = 0;
            dist[i][j] = Coordinates::WGS84::distance(lat1, lon1, hae1, lat2, lon2, hae2);
          }
        }

        // Solve TSP problem using dynamic programming
        double minCost = 100000;
        int prev;

        
        for (int i = 0; i < n_points; ++i){
          // Find cost of every path from node 0 to node i visiting every 
          // other node exactly once, before returning to node 0
          // Determine the cheapest one
          int subset = (1 << n_points) - 1; // Mask used to keep track of visited nodes
          double newCost = TSP(subset, i) + dist[i][0]; 
          if (newCost < minCost){
            // Keep track of index of cheapest second-to-last node
            prev = i;
          }
          minCost = std::min(minCost, newCost);
        }

        // Reconstruct shortest path
        int id_mask = (1 << n_points) - 1;
        std::vector<int> path_idx; // Final path indices
        path_idx.push_back(prev);
        for (int i = 0; i < n_points-2; i++){
          path_idx.push_back(path_indices[prev][id_mask]);
          id_mask = id_mask & (~(1 << prev));
          prev = path_idx.back();
        }
        path_idx.push_back(0);

        // Print path in console
        debugPrintPath(path_idx);

      }


      double 
      TSP(int subset, int i)
      {
        if (subset == ((1 << i) | 1)){
          // If subset is {1, i}, return distance from 1 to i
          return dist[0][i];
        }
        
        if (min_distances[i][subset] != 0){
          // If C(S, i) is already found, return C(S,i)
          return min_distances[i][subset];
        }
        
        double minCost = 100000;
        int prev = 0;
        
        for (int j = 0; j < n_points; ++j){
          // If j is not already visited, not equal to i and not the starting node
          if ((subset & (1 << j)) && j!=i && j!= 0){
            // Calculate C(S-{i},j)
            double newCost = TSP((subset & (~(1 << i))), j) + dist[j][i];
            if (newCost < minCost){
              prev = j;
            }
            minCost = std::min(minCost, newCost);
          }
        }
        // Save index of previous node in cheapest route
        path_indices[i][subset] = prev;
        // Memoize cheapest route from node 0 to node i
        min_distances[i][subset] = minCost;
        return minCost;
      }


      void debugPrintPath(std::vector<int> path)
      {
        std::string pathstr = "Planned path: ";
        for (int i = 0; i<path.size(); i++){
          pathstr = pathstr + std::to_string(path[i]);
          if (i != path.size() - 1){
            pathstr = pathstr + "->";
          }
        }
        spew(pathstr.c_str());
      }

    };
  }
}

DUNE_TASK
