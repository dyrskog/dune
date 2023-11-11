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
        while (!stopping())
        {
          waitForMessages(1.0);
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

    };
  }
}

DUNE_TASK
